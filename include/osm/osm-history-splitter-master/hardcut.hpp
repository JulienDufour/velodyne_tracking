#ifndef SPLITTER_HARDCUT_HPP
#define SPLITTER_HARDCUT_HPP

#include <vector>

#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/memory/buffer.hpp>

#include "cut.hpp"
#include "growing_bitset.hpp"

/*

Hardcut Algorithm
 - walk over all node-versions
   - walk over all bboxes
     - if node-writing for this bbox is still disabled
       - if the node-version is in the bbox
         - write the node to this bboxes writer
         - record its id in the bboxes node-id-tracker



 - walk over all way-versions
   - walk over all bboxes
     - create a new way NULL pointer
     - walk over all waynodes
       - if the waynode is in the node-id-tracker of this bbox
         - if the new way pointer is NULL
           - create a new way with all meta-data and tags but without waynodes
         - add the waynode to the new way

     - if the way pointer is not NULL
       - if the way has <2 waynodes
         - continue with the next way-version
       - write the way to this bboxes writer
       - record its id in the bboxes way-id-tracker



 - walk over all relation-versions
   - walk over all bboxes
     - create a new relation NULL pointer
     - walk over all relation members
       - if the relation member is in the node-id-tracker or the way-id-tracker of this bbox
         - if the new relation pointer is NULL
           - create a new relation with all meta-data and tags but without members
         - add the member to the new relation

     - if the relation pointer is not NULL
       - write the relation to this bboxes writer

features:
 - single pass
 - ways are cropped at bbox boundaries
 - relations contain only members that exist in the file
 - ways and relations are reference-complete
 - needs (theroeticvally) only ~182,4 MB RAM per extract (practically ~190 MB RAM)
   - ((1400000000÷8)+(130000000÷8))÷1024÷1024
   - 1.4 mrd nodes & 130 mio ways, one bit each, in megabytes

disadvantages:
 - relations referring to relations that come later in the file are missing this valid references
 - ways that have only one node inside the bbox are missing from the output
 - only versions of an object that are inside the bboxes are in thr extract, some versions may be missing

*/

class HardcutExtractInfo : public ExtractInfo {

public:
    growing_bitset node_tracker;
    growing_bitset way_tracker;

    HardcutExtractInfo(const std::string& name, const osmium::io::File& file, const osmium::io::Header& header) :
        ExtractInfo(name, file, header) {}
};

class HardcutInfo : public CutInfo<HardcutExtractInfo> {

};

class Hardcut : public Cut<HardcutInfo> {

    void copy_tags(osmium::memory::Buffer& buffer, osmium::builder::Builder& builder, const osmium::TagList& tags) {
        osmium::builder::TagListBuilder tl_builder(buffer, &builder);
        for (const auto& tag : tags) {
            tl_builder.add_tag(tag.key(), tag.value());
        }
    }

public:

    Hardcut(HardcutInfo *info) : Cut<HardcutInfo>(info) {
        if (debug) {
            std::cerr << "hardcut init\n";
        }

        for (const auto& extract : info->extracts) {
            std::cerr << "\textract " << extract->name << "\n";
        }

        if (debug) {
            std::cerr << "\n\n===== NODES =====\n\n";
        }
    }

    void node(const osmium::Node& node) {
        if (debug) {
            std::cerr << "hardcut node " << node.id() << " v" << node.version() << "\n";
        }

        for (const auto& extract : info->extracts) {
            if (extract->contains(node)) {
                if (debug) {
                    std::cerr << "node " << node.id() << " v" << node.version() << " is inside bbox, writing it out\n";
                }

                extract->write(node);

                extract->node_tracker.set(node.id());
            }
        }
    }

    void way(const osmium::Way& way) {
        if (debug) {
            std::cerr << "hardcut way " << way.id() << " v" << way.version() << "\n";
        }

        std::vector<osmium::object_id_type> node_ids;
        node_ids.reserve(way.nodes().size());

        for (const auto& extract : info->extracts) {
            node_ids.clear();

            for (const auto& node_ref : way.nodes()) {
                if (extract->node_tracker.get(node_ref.ref())) {
                    if (debug) {
                        std::cerr << "adding node-id " << node_ref.ref() << " to cutted way " << way.id() << " v" << way.version() << " for bbox\n";
                    }
                    node_ids.push_back(node_ref.ref());
                }
            }

            if (!node_ids.empty()) {
                osmium::memory::Buffer buffer(1024*1024, osmium::memory::Buffer::auto_grow::yes);

                {
                    osmium::builder::WayBuilder builder(buffer);

                    if (debug) {
                        std::cerr << "creating cutted way " << way.id() << " v" << way.version() << " for bbox\n";
                    }

                    auto& newway = builder.object();
                    newway.set_id(way.id());
                    newway.set_version(way.version());
                    newway.set_uid(way.uid());
                    newway.set_changeset(way.changeset());
                    newway.set_timestamp(way.timestamp());
                    newway.set_visible(way.visible());

                    builder.add_user(way.user());

                    copy_tags(buffer, builder, way.tags());

                    {
                        osmium::builder::WayNodeListBuilder wnl_builder{buffer, &builder};
                        for (auto id : node_ids) {
                            wnl_builder.add_node_ref(id);
                        }
                    }
                }

                buffer.commit();

                if (debug) {
                    std::cerr << "way " << way.id() << " v" << way.version() << " is in bbox\n";
                }

                // check for short ways
                const osmium::Way& newway = buffer.get<osmium::Way>(0);
                if (newway.nodes().size() < 2) {
                    if (debug) {
                        std::cerr << "way " << way.id() << " v" << way.version() << " in bbox would only be " << newway.nodes().size() << " nodes long, skipping\n";
                    }
                    continue;
                }

                if (debug) {
                    std::cerr << "way " << way.id() << " v" << way.version() << " is inside bbox, writing it out\n";
                }
                extract->write(newway);

                extract->way_tracker.set(way.id());
            }
        }
    }

    void relation(const osmium::Relation& relation) {
        if (debug) {
            std::cerr << "hardcut relation " << relation.id() << " v" << relation.version() << "\n";
        }

        std::vector<const osmium::RelationMember*> members;
        members.reserve(relation.members().size());

        for (const auto& extract : info->extracts) {
            members.clear();

            for (const auto& member : relation.members()) {
                if ((member.type() == osmium::item_type::node && extract->node_tracker.get(member.ref())) ||
                   (member.type() == osmium::item_type::way  && extract->way_tracker.get(member.ref()))) {
                    members.push_back(&member);
                }
            }

            if (!members.empty()) {
                osmium::memory::Buffer buffer(1024*1024, osmium::memory::Buffer::auto_grow::yes);

                {
                    osmium::builder::RelationBuilder builder(buffer);

                    if (debug) {
                        std::cerr << "creating cutted relation " << relation.id() << " v" << relation.version() << " for bbox\n";
                    }

                    auto& newrelation = builder.object();
                    newrelation.set_id(relation.id());
                    newrelation.set_version(relation.version());
                    newrelation.set_uid(relation.uid());
                    newrelation.set_changeset(relation.changeset());
                    newrelation.set_timestamp(relation.timestamp());
                    newrelation.set_visible(relation.visible());

                    builder.add_user(relation.user());

                    copy_tags(buffer, builder, relation.tags());

                    {
                        osmium::builder::RelationMemberListBuilder rml_builder{buffer, &builder};
                        for (auto memptr : members) {
                            rml_builder.add_member(memptr->type(), memptr->ref(), memptr->role());
                        }
                    }
                }

                buffer.commit();

                if (debug) {
                    std::cerr << "relation " << relation.id() << " v" << relation.version() << " is inside bbox, writing it out\n";
                }

                extract->write(buffer.get<osmium::Relation>(0));
            }
        }
    }

};

#endif // SPLITTER_HARDCUT_HPP
