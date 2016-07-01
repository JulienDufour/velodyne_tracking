#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <getopt.h>
#include <string>
#include <unistd.h>

#include <osmium/io/any_input.hpp>
#include <osmium/io/file.hpp>
#include <osmium/io/reader.hpp>

#include <geos/geom/MultiPolygon.h>
#include <geos/algorithm/locate/IndexedPointInAreaLocator.h>

#include "softcut.hpp"
#include "hardcut.hpp"

template <typename TExtractInfo>
bool readConfig(const std::string& conffile, CutInfo<TExtractInfo> &info) {
    const int linelen = 4096;

    FILE *fp = fopen(conffile.c_str(), "r");
    if (!fp) {
        std::cerr << "unable to open config file " << conffile << "\n";
        return false;
    }

    char line[linelen];
    while (fgets(line, linelen-1, fp)) {
        line[linelen-1] = '\0';
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n' || line[0] == '\0')
            continue;

        int n = 0;
        char *tok = strtok(line, "\t ");

        const char *name = nullptr;
        double minlon = 0, minlat = 0, maxlon = 0, maxlat = 0;
        char type = '\0';
        char file[linelen];

        while (tok) {
            switch(n) {
                case 0:
                    name = tok;
                    break;

                case 1:
                    if (0 == strcmp("BBOX", tok))
                        type = 'b';
                    else if (0 == strcmp("POLY", tok))
                        type = 'p';
                    else if (0 == strcmp("OSM", tok))
                        type = 'o';
                    else {
                        type = '\0';
                        std::cerr << "output " << name << " of type " << tok << ": unknown output type\n";
                        return false;
                    }
                    break;

                case 2:
                    switch(type) {
                        case 'b':
                            if (4 == sscanf(tok, "%lf,%lf,%lf,%lf", &minlon, &minlat, &maxlon, &maxlat)) {
                                info.addExtract(name, minlat, minlon, maxlat, maxlon);
                            } else {
                                std::cerr << "error reading BBOX " << tok << " for " << name << "\n";
                                return false;
                            }
                            break;
                        case 'p':
                            if (1 == sscanf(tok, "%s", file)) {
                                geos::geom::Geometry *geom = OsmiumExtension::GeometryReader::fromPolyFile(file);
                                if (!geom) {
                                    std::cerr << "error creating geometry from poly-file " << file << " for " << name << "\n";
                                    break;
                                }
                                info.addExtract(name, geom);
                            }
                            break;
                        case 'o':
                            if (1 == sscanf(tok, "%s", file)) {
                                geos::geom::Geometry *geom = OsmiumExtension::GeometryReader::fromOsmFile(file);
                                if (!geom) {
                                    std::cerr << "error creating geometry from poly-file " << file << " for " << name << "\n";
                                    break;
                                }
                                info.addExtract(name, geom);
                            }
                            break;
                    }
                    break;
            }

            tok = strtok(nullptr, "\t ");
            n++;
        }
    }
    fclose(fp);
    return true;
}

int main(int argc, char *argv[]) {

    bool softcut = true;
    bool debug = false;

    static struct option long_options[] = {
        {"debug",   no_argument, 0, 'd'},
        {"softcut", no_argument, 0, 's'},
        {"hardcut", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while (true) {
        int c = getopt_long(argc, argv, "dsh", long_options, 0);
        if (c == -1)
            break;

        switch (c) {
            case 'd':
                debug = true;
                break;
            case 's':
                softcut = true;
                break;
            case 'h':
                softcut = false;
                break;
        }
    }

    if (optind > argc-2) {
        std::cerr << "Usage: " << argv[0] << " [OPTIONS] OSMFILE CONFIGFILE\n";
        return 1;
    }

    std::string filename{argv[optind]};
    std::string conffile{argv[optind+1]};

    if (softcut && filename == "-") {
        std::cerr << "Can't read from stdin when in softcut\n";
        return 1;
    }

    osmium::io::File infile(filename);

    if (softcut) {
        SoftcutInfo info;
        if (!readConfig(conffile, info)) {
            std::cerr << "error reading config\n";
            return 1;
        }

        {
            SoftcutPassOne one(&info);
            one.debug = debug;
            osmium::io::Reader reader(infile);
            osmium::apply(reader, one);
            reader.close();
        }

        {
            SoftcutPassTwo two(&info);
            two.debug = debug;
            osmium::io::Reader reader(infile);
            osmium::apply(reader, two);
            reader.close();
        }
    } else {
        HardcutInfo info;
        if (!readConfig(conffile, info)) {
            std::cerr << "error reading config\n";
            return 1;
        }

        Hardcut cutter(&info);
        cutter.debug = debug;
        osmium::io::Reader reader(infile);
        osmium::apply(reader, cutter);
        reader.close();
    }
    
    return 0;
}

