#ifndef SPLITTER_CUT_HPP
#define SPLITTER_CUT_HPP

#include <osmium/io/any_output.hpp>

#include "geometryreader.hpp"

// information about a single extract
class ExtractInfo {

public:
    enum ExtractMode {
        LOCATOR = 1,
        BOUNDS = 2
    };

    std::string name;
    geos::algorithm::locate::IndexedPointInAreaLocator *locator;
    osmium::Box bounds;
    osmium::io::Writer writer;
    ExtractMode mode;
    osmium::memory::Buffer m_buffer;

    ExtractInfo(const std::string& name, const osmium::io::File& file, const osmium::io::Header& header) :
        locator(nullptr),
        writer(file, header),
        m_buffer(1024*1024, osmium::memory::Buffer::auto_grow::yes) {
        this->name = name;
    }

    ~ExtractInfo() {
        flush();
        writer.close();
        if (locator) delete locator;
    }

    bool contains(const osmium::Node& node) {
        if (mode == BOUNDS) {
            return
                (node.location().lon() > bounds.bottom_left().lon()) &&
                (node.location().lat() > bounds.bottom_left().lat()) &&
                (node.location().lon() < bounds.top_right().lon()) &&
                (node.location().lat() < bounds.top_right().lat());
        } else if (mode == LOCATOR) {
            // BOUNDARY 1
            // EXTERIOR 2
            // INTERIOR 0

            geos::geom::Coordinate c = geos::geom::Coordinate(node.location().lon(), node.location().lat(), DoubleNotANumber);
            return (0 == locator->locate(&c));
        }

        return false;
    }

    void flush() {
        osmium::memory::Buffer new_buffer(1024*1024, osmium::memory::Buffer::auto_grow::yes);
        using std::swap;
        swap(m_buffer, new_buffer);
        writer(std::move(new_buffer));
    }

    void write(const osmium::OSMObject& object) {
        m_buffer.add_item(object);
        m_buffer.commit();
        if (m_buffer.committed() > 900 * 1024) {
            flush();
        }
    }

};

// information about the cutting algorithm
template <class TExtractInfo>
class CutInfo {

protected:
    ~CutInfo() {
        for (auto& extract : extracts) {
            delete extract;
        }
    }

public:
    std::vector<TExtractInfo*> extracts;

    TExtractInfo *addExtract(const std::string& name, double minlon, double minlat, double maxlon, double maxlat) {
        std::cerr << "opening writer for " << name.c_str() << "\n";
        osmium::io::File outfile(name);

        const osmium::Location min(minlat, minlon);
        const osmium::Location max(maxlat, maxlon);

        osmium::Box bounds;
        bounds.extend(min).extend(max);

        osmium::io::Header header;
        header.add_box(bounds);

        TExtractInfo *ex = new TExtractInfo(name, outfile, header);
        ex->bounds = bounds;
        ex->mode = ExtractInfo::BOUNDS;

        extracts.push_back(ex);
        return ex;
    }

    TExtractInfo *addExtract(const std::string& name, geos::geom::Geometry *poly) {
        std::cerr << "opening writer for " << name.c_str() << "\n";
        osmium::io::File outfile(name);

        const geos::geom::Envelope *env = poly->getEnvelopeInternal();
        const osmium::Location min(env->getMinX(), env->getMinY());
        const osmium::Location max(env->getMaxX(), env->getMaxY());

        osmium::Box bounds;
        bounds.extend(min).extend(max);

        osmium::io::Header header;
        header.add_box(bounds);

        TExtractInfo *ex = new TExtractInfo(name, outfile, header);
        ex->locator = new geos::algorithm::locate::IndexedPointInAreaLocator(*poly);
        ex->mode = ExtractInfo::LOCATOR;

//XXX        Osmium::Geometry::geos_geometry_factory()->destroyGeometry(poly);

        extracts.push_back(ex);
        return ex;
    }
};

template <class TCutInfo>
class Cut : public osmium::handler::Handler {

protected:

    TCutInfo *info;

public:

    bool debug;
    Cut(TCutInfo *info) : info(info) {}
};

#endif // SPLITTER_CUT_HPP

