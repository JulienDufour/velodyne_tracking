#ifndef GROWING_BITSET_HPP
#define GROWING_BITSET_HPP

#include <memory>
#include <vector>

#include <osmium/osm/types.hpp>

class growing_bitset {

    static const size_t segment_size = 50*1024*1024;

    typedef std::vector<bool> bitvec_type;
    typedef bitvec_type* bitvec_ptr_type;

    std::vector<std::unique_ptr<bitvec_type>> bitmap;

    static size_t segment(const osmium::object_id_type pos) {
        return pos / static_cast<osmium::object_id_type>(segment_size);
    }

    static size_t segmented_pos(const osmium::object_id_type pos) {
        return pos % static_cast<osmium::object_id_type>(segment_size);
    }

    bitvec_ptr_type find_segment(size_t segment) {
        if (segment >= bitmap.size()) {
            bitmap.resize(segment+1);
        }

        auto ptr = bitmap[segment].get();
        if (!ptr) {
            bitmap[segment].reset(new bitvec_type(segment_size));
            return bitmap[segment].get();
        }

        return ptr;
    }

    bitvec_ptr_type find_segment(size_t segment) const {
        if (segment >= bitmap.size()) {
            return nullptr;
        }

        return bitmap[segment].get();
    }

public:

    void set(const osmium::object_id_type pos) {
        bitvec_ptr_type bitvec = find_segment(segment(pos));
        bitvec->at(segmented_pos(pos)) = true;
    }

    bool get(const osmium::object_id_type pos) const {
        bitvec_ptr_type bitvec = find_segment(segment(pos));
        if (!bitvec) return false;
        return bitvec->at(segmented_pos(pos));
    }

    void clear() {
        for (auto& ptr : bitmap) {
            ptr->clear();
        }
    }

}; // class growing_bitset

#endif
