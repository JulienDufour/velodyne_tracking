#ifndef OSMIUMEX_GEOMBUILDER_HPP
#define OSMIUMEX_GEOMBUILDER_HPP

#include <cstring>

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/MultiPolygon.h>
#include <geos/geom/Point.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/util/GEOSException.h>

#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/dense_mem_array.hpp>
#include <osmium/io/file.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/visitor.hpp>


typedef osmium::index::map::DenseMemArray<osmium::unsigned_object_id_type, osmium::Location> storage_array_t;
typedef osmium::handler::NodeLocationsForWays<storage_array_t, storage_array_t> cfw_handler_t;

namespace OsmiumExtension {

    geos::geom::GeometryFactory* geos_factory() {
        static std::unique_ptr<const geos::geom::PrecisionModel> precision_model{new geos::geom::PrecisionModel};
        static std::unique_ptr<geos::geom::GeometryFactory> factory{new geos::geom::GeometryFactory(precision_model.get(), -1)};
        return factory.get();
    }

    class OsmGeometryReader : public osmium::handler::Handler {
        std::vector<geos::geom::Geometry*> outer;
        storage_array_t store_pos;
        storage_array_t store_neg;
        cfw_handler_t* handler_cfw;

    private:
            geos::geom::Geometry *polygonFromWay(const osmium::Way& way) const {
                if (!way.is_closed()) {
                    std::cerr << "can't build way polygon geometry of unclosed way, leave it as nullptr\n";
                    return nullptr;
                }
                try {
                    std::vector<geos::geom::Coordinate> *c = new std::vector<geos::geom::Coordinate>;

                    for (const auto& node_ref : way.nodes()) {
                        c->emplace_back(node_ref.location().lon(), node_ref.location().lat());
                    }

                    geos::geom::CoordinateSequence *cs = geos_factory()->getCoordinateSequenceFactory()->create(c);
                    geos::geom::LinearRing *ring = geos_factory()->createLinearRing(cs);
                    return static_cast<geos::geom::Geometry *>(geos_factory()->createPolygon(ring, nullptr));
                } catch (const geos::util::GEOSException& exc) {
                    std::cerr << "error building way geometry, leave it as nullptr\n";
                    return nullptr;
                }
            }

    public:
        OsmGeometryReader() :
            Handler(),
            store_pos(5000),
            store_neg(1000) {
            handler_cfw = new cfw_handler_t(store_pos, store_neg);
        }

        virtual ~OsmGeometryReader() {
            delete handler_cfw;

            for (uint32_t i=0; i < outer.size(); i++) {
                geos_factory()->destroyGeometry(outer[i]);
            }
            outer.clear();
        }

        void node(const osmium::Node& node) {
            handler_cfw->node(node);
        }

        void way(osmium::Way& way) {
            handler_cfw->way(way);

            if (!way.is_closed()) {
                std::cerr << "open way " << way.id() << " in osm-input\n";
                return;
            }

            geos::geom::Geometry *geom = polygonFromWay(way);
            if (!geom) {
                std::cerr << "error creating polygon from way\n";
                return;
            }
            outer.push_back(geom);
        }

        geos::geom::Geometry *buildGeom() const {
            geos::geom::MultiPolygon *outerPoly;
            try {
                outerPoly = geos_factory()->createMultiPolygon(outer);
            } catch(geos::util::GEOSException e) {
                std::cerr << "error creating multipolygon: " << e.what() << "\n";
                return nullptr;
            }
            return outerPoly;
        }
    };

    class GeometryReader {

        /// maximum length of a line in a .poly file
        static const int polyfile_linelen = 2048;

    public:

        /**
         * read a .poly file and generate a geos Geometry from it.
         *
         * .poly-files are simple:
         *   - a title
         *   - 1..n polygons
         *     - a polygon number, possibly with a ! in front so signalize holes
         *     - 1..n lines with floating point latitudes and longitudes
         *     - END token
         *   - END token
         *
         * usually the Geometry read from .poly files are used together with an
         * geos::algorithm::locate::IndexedPointInAreaLocator to check for nodes
         * being located inside the polygon.
         *
         * this method returns nullptr if the .poly file can't be read.
         */
        static geos::geom::Geometry *fromPolyFile(const std::string &file) {

            // pointer to coordinate vector
            std::vector<geos::geom::Coordinate> *c = nullptr;

            // vectors of outer and inner polygons
            std::vector<geos::geom::Geometry*> *outer = new std::vector<geos::geom::Geometry*>();
            std::vector<geos::geom::Geometry*> *inner = new std::vector<geos::geom::Geometry*>();

            // file pointer to .poly file
            FILE *fp = fopen(file.c_str(), "r");
            if (!fp) {
                std::cerr << "unable to open polygon file " << file << "\n";
                return nullptr;
            }

            // line buffer
            char line[polyfile_linelen];

            // read title line
            if (!fgets(line, polyfile_linelen-1, fp)) {
                std::cerr << "unable to read title line from polygon file " << file << "\n";
                return nullptr;
            }
            line[polyfile_linelen-1] = '\0';

            // is this polygon an inner polygon
            bool isinner = false;

            // are we currently inside parsing one polygon
            bool ispoly = false;

            // double x / y coords
            double x = 0, y = 0;

            // read through the file
            while (!feof(fp)) {
                // read a line
                if (!fgets(line, polyfile_linelen-1, fp)) {
                    std::cerr << "unable to read line from polygon file " << file << "\n";
                    return nullptr;
                }
                line[polyfile_linelen-1] = '\0';

                // when we're currently outside a polygon
                if (!ispoly) {
                    // if this is an end-line
                    if (0 == strncmp(line, "END", 3)) {
                        // cancel parsing
                        break;
                    }

                    // this is considered a polygon-start line
                    // if it begins with ! it signales the start of an inner polygon
                    isinner = (line[0] == '!');

                    // remember we're inside a polygon
                    ispoly = true;

                    // create a new coordinate sequence
                    c = new std::vector<geos::geom::Coordinate>();

                // when we're currently inside a polygon
                } else {
                    // if this is an end-line
                    if (0 == strncmp(line, "END", 3)) {
                        if (!c) {
                            std::cerr << "empty polygon file\n";
                            return nullptr;
                        }

                        // check if the polygon is closed
                        if (c->front() != c->back()) {
                            std::cerr << "auto-closing unclosed polygon\n";
                            c->push_back(c->front());
                        }

                        // build a polygon from the coordinate vector
                        geos::geom::Geometry* poly;
                        try {
                            poly = geos_factory()->createPolygon(
                                geos_factory()->createLinearRing(
                                    geos_factory()->getCoordinateSequenceFactory()->create(c)
                                ),
                                nullptr
                            );
                        } catch(geos::util::GEOSException e) {
                            std::cerr << "error creating polygon: " << e.what() << "\n";
                            return nullptr;
                        }

                        // add it to the appropriate polygon vector
                        if (isinner) {
                            inner->push_back(poly);
                        } else {
                            outer->push_back(poly);
                        }

                        // remember we're now outside a polygon
                        ispoly = false;

                    // an ordinary line
                    } else {
                        // try to parse it using sscanf
                        if (2 != sscanf(line, " %lE %lE", &x, &y)) {
                            std::cerr << "unable to parse line from polygon file " << file << ": " << line;
                            return nullptr;
                        }

                        // push the parsed coordinate into the coordinate vector
                        c->push_back(geos::geom::Coordinate(x, y, DoubleNotANumber));
                    }
                }
            }

            // check that the file ended with END
            if (0 != strncmp(line, "END", 3)) {
                std::cerr << "polygon file " << file << " does not end with END token\n";
                return nullptr;
            }

            // close the file pointer
            fclose(fp);

            // build MultiPolygons from the vectors of outer and inner polygons
            geos::geom::Geometry *poly;
            try {
                geos::geom::MultiPolygon *outerPoly = geos_factory()->createMultiPolygon(outer);
                geos::geom::MultiPolygon *innerPoly = geos_factory()->createMultiPolygon(inner);

                // generate a MultiPolygon containing the difference of those two
                poly = outerPoly->difference(innerPoly);

                // destroy the both MultiPolygons
                geos_factory()->destroyGeometry(outerPoly);
                geos_factory()->destroyGeometry(innerPoly);
            } catch(geos::util::GEOSException e) {
                std::cerr << "error creating differential multipolygon: " << e.what() << "\n";
                return nullptr;
            }

            // and return their difference
            return poly;
        } // fromPolyFile

        static geos::geom::Geometry *fromOsmFile(const std::string &file) {
            osmium::io::File infile(file);
            OsmiumExtension::OsmGeometryReader reader_handler;
            osmium::io::Reader reader(infile);
            osmium::apply(reader, reader_handler);
            geos::geom::Geometry *geom = reader_handler.buildGeom();

            return geom;
        }

        /**
         * construct a geos Geometry from a BoundingBox string.
         *
         * this method returns nullptr if the string can't be read.
         */
        static geos::geom::Geometry *fromBBox(const std::string &bbox) {
            double minlon, minlat, maxlon, maxlat;
            if (4 != sscanf(bbox.c_str(), "%lf,%lf,%lf,%lf", &minlon, &minlat, &maxlon, &maxlat)) {
                std::cerr << "invalid BBox string: " << bbox << "\n";
                return nullptr;
            }

            // build the Geometry from the coordinates
            return fromBBox(minlon, minlat, maxlon, maxlat);
        }

        static geos::geom::Geometry *fromBBox(double minlon, double minlat, double maxlon, double maxlat) {
            // create an Envelope and convert it to a polygon
            geos::geom::Envelope *e = new geos::geom::Envelope(minlon, maxlon, minlat, maxlat);
            geos::geom::Geometry *p = geos_factory()->toGeometry(e);

            delete e;
            return p;
        }

    }; // class GeomBuilder

} // namespace OsmiumExtension

#endif // OSMIUMEX_GEOMBUILDER_HPP
