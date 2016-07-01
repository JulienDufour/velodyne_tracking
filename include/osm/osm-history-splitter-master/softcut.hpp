#ifndef SPLITTER_SOFTCUT_HPP
#define SPLITTER_SOFTCUT_HPP

#include "cut.hpp"
#include "growing_bitset.hpp"

/*

Softcut Algorithm
 - walk over all node-versions
   - walk over all bboxes
     - if the current node-version is inside the bbox
       - record its id in the bboxes node-tracker

 - initialize the current-way-id to 0
 - walk over all way-versions
   - if current-way-id != 0 and current-way-id != the id of the currently iterated way (in other words: this is a different way)
     - walk over all bboxes
       - if the way-id is in the bboxes way-id-tracker (in other words: the way is in the output)
         - append all nodes of the current-way-nodes set to the extra-node-tracker
     - clear the current-way-nodes set
   - update the current-way-id
   - walk over all way-nodes
     - append the node-ids to the current-way-nodes set
   - walk over all bboxes
     - walk over all way-nodes
       - if the way-node is recorded in the bboxes node-tracker
         - record its id in the bboxes way-id-tracker

- after all ways
  - walk over all bboxes
    - if the way-id is in the bboxes way-id-tracker (in other words: the way is in the output)
      - append all nodes of the current-way-nodes set to the extra-node-tracker

 - walk over all relation-versions
   - walk over all bboxes
     - walk over all relation-members
       - if the relation-member is recorded in the bboxes node- or way-tracker
         - record its id in the bboxes relation-tracker

Second Pass
 - walk over all node-versions
   - walk over all bboxes
     - if the node-id is recorded in the bboxes node-tracker or in the extra-node-tracker
       - send the node to the bboxes writer

 - walk over all way-versions
   - walk over all bboxes
     - if the way-id is recorded in the bboxes way-tracker
       - send the way to the bboxes writer

 - walk over all relation-versions
   - walk over all bboxes
     - if the relation-id is recorded in the bboxes relation-tracker
       - send the relation to the bboxes writer

features:
 - if an object is in the extract, all versions of it are there
 - ways and relations are not changed
 - ways are reference-complete

disadvantages
 - dual pass
 - needs more RAM: 350 MB per BBOX
   - ((1400000000÷8)+(1400000000÷8)+(130000000÷8)+(1500000÷8))÷1024÷1024 MB
 - relations will have dead references

*/


class SoftcutExtractInfo : public ExtractInfo {

public:
    growing_bitset node_tracker;
    growing_bitset extra_node_tracker;
    growing_bitset way_tracker;
    growing_bitset relation_tracker;

    SoftcutExtractInfo(const std::string& name, const osmium::io::File& file, const osmium::io::Header& header) :
        ExtractInfo(name, file, header) {}
};

class SoftcutInfo : public CutInfo<SoftcutExtractInfo> {

public:
    std::multimap<osmium::object_id_type, osmium::object_id_type> cascading_relations_tracker;

};


class SoftcutPassOne : public Cut<SoftcutInfo> {

    osmium::object_id_type current_way_id;
    std::set<osmium::object_id_type> current_way_nodes;
    bool first_relation = true;

    // - walk over all bboxes
    //   - if the way-id is in the bboxes way-id-tracker (in other words: the way is in the output)
    //     - append all nodes of the current-way-nodes set to the extra-node-tracker
    void write_way_extra_nodes() {
        if (debug) {
            std::cerr << "finished all versions of way " << current_way_id << ", checking for extra nodes\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->way_tracker.get(current_way_id)) {
                if (debug) {
                    std::cerr << "way had a node inside extract, recording extra nodes\n";
                }

                for (const auto id : current_way_nodes) {
                    extract->extra_node_tracker.set(id);
                    if (debug) {
                        std::cerr << "  " << id;
                    }
                }

                if (debug) {
                    std::cerr << "\n";
                }
            }
        }
    }

public:

    SoftcutPassOne(SoftcutInfo *info) : Cut<SoftcutInfo>(info), current_way_id(0), current_way_nodes() {
        std::cerr << "softcut first-pass init\n";
        for (const auto& extract : info->extracts) {
            std::cerr << "\textract " << extract->name << "\n";
        }

        if (debug) {
            std::cerr << "\n\n===== NODES =====\n\n";
        }
    }

    // - walk over all node-versions
    //   - walk over all bboxes
    //     - if the current node-version is inside the bbox
    //       - record its id in the bboxes node-tracker
    void node(const osmium::Node& node) {
        if (debug) {
            std::cerr << "softcut node " << node.id() << " v" << node.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->contains(node)) {
                if (debug) std::cerr << "node is in extract, recording in node_tracker\n";

                extract->node_tracker.set(node.id());
            }
        }
    }

    // - initialize the current-way-id to 0
    // - walk over all way-versions
    //   - if current-way-id != 0 and current-way-id != the id of the currently iterated way (in other words: this is a different way)
    //     - walk over all bboxes
    //       - if the way-id is in the bboxes way-id-tracker (in other words: the way is in the output)
    //         - append all nodes of the current-way-nodes set to the extra-node-tracker
    //     - clear the current-way-nodes set
    //   - update the current-way-id
    //   - walk over all way-nodes
    //     - append the node-ids to the current-way-nodes set
    //   - walk over all bboxes
    //     - walk over all way-nodes
    //       - if the way-node is recorded in the bboxes node-tracker
    //         - record its id in the bboxes way-id-tracker
    //
    // - after all ways
    //   - walk over all bboxes
    //     - if the way-id is in the bboxes way-id-tracker (in other words: the way is in the output)
    //       - append all nodes of the current-way-nodes set to the extra-node-tracker

    void way(const osmium::Way& way) {
        // detect a new way
        if (current_way_id != 0 && current_way_id != way.id()) {
            write_way_extra_nodes();
            current_way_nodes.clear();
        }
        current_way_id = way.id();

        if (debug) {
            std::cerr << "softcut way " << way.id() << " v" << way.version() << "\n";
        }

        for (const auto& node_ref : way.nodes()) {
            current_way_nodes.insert(node_ref.ref());
        }

        for (const auto& extract : info->extracts) {
            for (const auto& node_ref : way.nodes()) {
                if (extract->node_tracker.get(node_ref.ref())) {
                    if (debug) {
                        std::cerr << "way has a node (" << node_ref.ref() << ") inside extract, recording in way_tracker\n";
                    }
                    extract->way_tracker.set(way.id());
                    break;
                }
            }
        }
    }

    // - walk over all relation-versions
    //   - walk over all bboxes
    //     - walk over all relation-members
    //       - if the relation-member is recorded in the bboxes node- or way-tracker
    //         - record its id in the bboxes relation-tracker
    void relation(const osmium::Relation& relation) {
        if (first_relation) {
            write_way_extra_nodes();
            first_relation = false;
        }

        if (debug) {
            std::cerr << "softcut relation " << relation.id() << " v" << relation.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            bool hit = false;

            for (const auto& member : relation.members()) {

                if (!hit && (
                    (member.type() == osmium::item_type::node && extract->node_tracker.get(member.ref())) ||
                    (member.type() == osmium::item_type::way && extract->way_tracker.get(member.ref())) ||
                    (member.type() == osmium::item_type::relation && extract->relation_tracker.get(member.ref()))
                )) {

                    if (debug) std::cerr << "relation has a member (" << member.type() << " " << member.ref() << ") inside extract, recording in relation_tracker\n";
                    hit = true;

                    extract->relation_tracker.set(relation.id());
                }

                if (member.type() == osmium::item_type::relation) {
                    if (debug) {
                        std::cerr << "recording cascading-pair: " << member.ref() << " -> " << relation.id() << "\n";
                    }
                    info->cascading_relations_tracker.insert(std::make_pair(member.ref(), relation.id()));
                }
            }

            if (hit) {
                cascading_relations(extract, relation.id());
            }
        }
    }

    void cascading_relations(SoftcutExtractInfo *extract, osmium::object_id_type id) {
        auto r = info->cascading_relations_tracker.equal_range(id);

        for (auto it = r.first; it != r.second; ++it) {
            if (debug) std::cerr << "\tcascading: " << it->second << "\n";

            if (extract->relation_tracker.get(it->second)) {
                continue;
            }

            extract->relation_tracker.set(it->second);

            cascading_relations(extract, it->second);
        }
    }

}; // class SoftcutPassOne


class SoftcutPassTwo : public Cut<SoftcutInfo> {

public:

    SoftcutPassTwo(SoftcutInfo *info) : Cut<SoftcutInfo>(info) {
        if (debug) {
            std::cerr << "softcut second-pass init\n";
        }
    }

    // - walk over all node-versions
    //   - walk over all bboxes
    //     - if the node-id is recorded in the bboxes node-tracker or in the extra-node-tracker
    //       - send the node to the bboxes writer
    void node(const osmium::Node& node) {
        if (debug) {
            std::cerr << "softcut node " << node.id() << " v" << node.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->node_tracker.get(node.id()) ||
                extract->extra_node_tracker.get(node.id())) {
                extract->write(node);
            }
        }
    }

    // - walk over all way-versions
    //   - walk over all bboxes
    //     - if the way-id is recorded in the bboxes way-tracker
    //       - send the way to the bboxes writer
    void way(const osmium::Way& way) {
        if (debug) {
            std::cerr << "softcut way " << way.id() << " v" << way.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->way_tracker.get(way.id())) {
                extract->write(way);
            }
        }
    }

    // - walk over all relation-versions
    //   - walk over all bboxes
    //     - if the relation-id is recorded in the bboxes relation-tracker
    //       - send the relation to the bboxes writer
    void relation(const osmium::Relation& relation) {
        if (debug) {
            std::cerr << "softcut relation " << relation.id() << " v" << relation.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->relation_tracker.get(relation.id())) {
                extract->write(relation);
            }
        }
    }

}; // class SoftcutPassTwo

#endif // SPLITTER_SOFTCUT_HPP

