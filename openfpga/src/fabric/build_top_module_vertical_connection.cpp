/**
 * File containing the functions to build the vertical connections between 3D SBs in the top module
 * Functions in this file are modified versions of some of the ones in build_top_module_connection.cpp
 */

/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_time.h"

/* Headers from openfpgashell library */
#include "command_exit_codes.h"

/* Headers from openfpgautil library */
#include "openfpga_side_manager.h"

/* Headers from vpr library */
#include "build_routing_module_utils.h"
#include "build_top_module_connection.h"
#include "build_top_module_utils.h"
#include "module_manager_utils.h"
#include "openfpga_device_grid_utils.h"
#include "openfpga_naming.h"
#include "openfpga_physical_tile_utils.h"
#include "openfpga_reserved_words.h"
#include "openfpga_rr_graph_utils.h"
#include "pb_type_utils.h"
#include "rr_gsb_utils.h"
#include "vpr_utils.h"

namespace openfpga {
    void add_top_module_nets_connect_sb_and_sb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer) {

            /**
             * Goal of function is to connect the vertical tracks of the switch blocks in the top module
             * If the switch block is on layer 0 then there's only the above channel where the SB at layer 0 is the source 
             * and the SB at layer 1 is the sink for the above_out_TSV track
             * 
             * If the switch block is on the last layer then there's only the below channel where the SB at the last layer is the source
             * and the SB at the second last layer is the sink for the below_out_TSV track
             * 
             * If the switch block is on any other layer then there are both above and below channels 
             * where the SB at the current layer is the source and the SB at the layer above is the sink for the above_out_TSV track
             * and the SB at the layer below is the sink for the below_out_TSV track
             * 
             * In this function each SB only adds the nets for the channels in which it is the source, since each SB is looped over
             * the nets for all channels are added to the top module eventually
             */
            
            // VTR_LOG("Adding interlayer SB connections for SB[%lu][%lu][%lu] on layer %lu\n", rr_gsb.get_sb_x(), rr_gsb.get_sb_y(), layer, layer);

            vtr::Point<size_t> sb_coordinate = rr_gsb.get_sb_coordinate();

            if (true == compact_routing_hierarchy){
                const RRGSB& unique_mirror = device_rr_gsb.get_sb_unique_module(sb_coordinate, layer);
                sb_coordinate.set_x(unique_mirror.get_sb_x());
                sb_coordinate.set_y(unique_mirror.get_sb_y());
            }

            std::string sb_module_name = generate_switch_block_module_name(sb_coordinate, layer);

            ModuleId sb_module_id = module_manager.find_module(sb_module_name);

            size_t sb_instance_id = sb_instance_ids[layer][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

            // ASSUMPTION BEING MADE:
            // 1. Every Switch Block has an above and under port
            // 2. The Switch in the other layer is directly above or below the current switch block
            // 3. That switch block has the same port size as the current switch block

            // TODO: Add a check for the above assumptions, and skip the SB if they are not met

            if (layer == 0){ // only worry about "above_out"

                if (rr_gsb.get_chan_width(e_side::ABOVE) == 0){
                    return;
                }

                vtr::Point<size_t> above_sb_coordinate = rr_gsb.get_sb_coordinate();
                
                if(true == compact_routing_hierarchy){
                    const RRGSB& above_sb_instance = device_rr_gsb.get_sb_unique_module(above_sb_coordinate, layer + 1);
                    above_sb_coordinate.set_x(above_sb_instance.get_sb_x());
                    above_sb_coordinate.set_y(above_sb_instance.get_sb_y());
                } 
                
                const RRGSB& above_sb = device_rr_gsb.get_gsb(above_sb_coordinate, layer + 1);

                std::string above_sb_module_name = generate_switch_block_module_name(above_sb_coordinate, layer + 1);

                std::string above_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::OUT_PORT);

                std::string under_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::IN_PORT);

                ModuleId above_sb_module_id = module_manager.find_module(above_sb_module_name);

                size_t above_sb_instance_id = sb_instance_ids[layer + 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, above_output_port_name);

                if (sb_port_id == ModulePortId::INVALID()){
                    return;
                }

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId above_sb_port_id = module_manager.find_module_port(above_sb_module_id, under_input_port_name);

                if (above_sb_port_id == ModulePortId::INVALID()){
                    VTR_ERROR_H("Invalid port id for port %s in module %s, the SB on the layer above has vertical connections while this module doesn't accept vertical connections.", under_input_port_name.c_str(), above_sb_module_name.c_str());
                }

                // connect the source SB to the sink SB (above_out to under_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, above_sb_module_id,
                                                    above_sb_instance_id, above_sb_port_id, itrack);
                }

            } else if (layer == device_rr_gsb.get_gsb_layers() - 1){ // only worry about "under_out"

                if (rr_gsb.get_chan_width(e_side::UNDER) == 0){
                    return;
                }

                vtr::Point<size_t> under_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& under_sb_instance = device_rr_gsb.get_sb_unique_module(under_sb_coordinate, layer - 1);
                    under_sb_coordinate.set_x(under_sb_instance.get_sb_x());
                    under_sb_coordinate.set_y(under_sb_instance.get_sb_y());
                }

                const RRGSB& under_sb = device_rr_gsb.get_gsb(under_sb_coordinate, layer - 1);
                std::string under_sb_module_name = generate_switch_block_module_name(under_sb_coordinate, layer - 1);

                std::string under_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::OUT_PORT);

                std::string above_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::IN_PORT);

                ModuleId under_sb_module_id = module_manager.find_module(under_sb_module_name);

                size_t under_sb_instance_id = sb_instance_ids[layer - 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, under_output_port_name);

                if (sb_port_id == ModulePortId::INVALID()){
                    return;
                }

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId under_sb_port_id = module_manager.find_module_port(under_sb_module_id, above_input_port_name); 

                if (under_sb_port_id == ModulePortId::INVALID()){
                    VTR_ERROR_H("Invalid port id for port %s in module %s, the SB on the layer below has vertical connections while this module doesn't accept vertical connections.", above_input_port_name.c_str(), under_sb_module_name.c_str());
                }

                // connect the source SB to the sink SB (under_out to above_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, under_sb_module_id,
                                                    under_sb_instance_id, under_sb_port_id, itrack);
                }


            } else{ // worry about both "above_out" and "under_out"

                if (rr_gsb.get_chan_width(e_side::ABOVE) == 0 && rr_gsb.get_chan_width(e_side::UNDER) == 0){
                    return;
                }

                vtr::Point<size_t> above_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& above_sb_instance = device_rr_gsb.get_sb_unique_module(above_sb_coordinate, layer + 1);
                    above_sb_coordinate.set_x(above_sb_instance.get_sb_x());
                    above_sb_coordinate.set_y(above_sb_instance.get_sb_y());
                }

                vtr::Point<size_t> under_sb_coordinate = rr_gsb.get_sb_coordinate();

                if(true == compact_routing_hierarchy){
                    const RRGSB& under_sb_instance = device_rr_gsb.get_sb_unique_module(under_sb_coordinate, layer - 1);
                    under_sb_coordinate.set_x(under_sb_instance.get_sb_x());
                    under_sb_coordinate.set_y(under_sb_instance.get_sb_y());
                }

                const RRGSB& above_sb = device_rr_gsb.get_gsb(above_sb_coordinate, layer + 1);
                std::string above_sb_module_name = generate_switch_block_module_name(above_sb_coordinate, layer + 1);

                const RRGSB& under_sb = device_rr_gsb.get_gsb(under_sb_coordinate, layer - 1);
                std::string under_sb_module_name = generate_switch_block_module_name(under_sb_coordinate, layer - 1);

                std::string above_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::OUT_PORT);
                std::string under_output_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::OUT_PORT);

                std::string above_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::ABOVE, PORTS::IN_PORT);;
                std::string under_input_port_name = generate_sb_module_track_port_name(CHANX, e_side::UNDER, PORTS::IN_PORT);

                ModuleId above_sb_module_id = module_manager.find_module(above_sb_module_name);
                ModuleId under_sb_module_id = module_manager.find_module(under_sb_module_name);

                size_t above_sb_instance_id = sb_instance_ids[layer + 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];
                size_t under_sb_instance_id = sb_instance_ids[layer - 1][rr_gsb.get_sb_x()][rr_gsb.get_sb_y()];

                ModulePortId sb_port_id = module_manager.find_module_port(sb_module_id, above_output_port_name);

                if (sb_port_id == ModulePortId::INVALID()){
                    return;
                }

                BasicPort sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId above_sb_port_id = module_manager.find_module_port(above_sb_module_id, under_input_port_name);

                if (above_sb_port_id == ModulePortId::INVALID()){
                    VTR_ERROR_H("Invalid port id for port %s in module %s, the SB on the layer below has vertical connections while this module doesn't accept vertical connections.", under_input_port_name.c_str(), above_sb_module_name.c_str());
                }

                // connect the source SB to the sink SB (above_out to under_in)

                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, above_sb_module_id,
                                                    above_sb_instance_id, above_sb_port_id, itrack);
                }

                sb_port_id = module_manager.find_module_port(sb_module_id, under_output_port_name);

                if (sb_port_id == ModulePortId::INVALID()){
                    return;
                }

                sb_port = module_manager.module_port(sb_module_id, sb_port_id);

                ModulePortId under_sb_port_id = module_manager.find_module_port(under_sb_module_id, above_input_port_name);

                if (under_sb_port_id == ModulePortId::INVALID()){
                    VTR_ERROR_H("Invalid port id for port %s in module %s, the SB on the layer above has vertical connections while this module doesn't accept vertical connections.", above_input_port_name.c_str(), under_sb_module_name.c_str());
                }

                // connect the source SB to the sink SB (under_out to above_in)
                for (size_t itrack = 0; itrack < sb_port.get_width(); ++itrack) {
                    ModuleNetId net =
                        create_module_source_pin_net(module_manager, top_module, sb_module_id,
                                                    sb_instance_id, sb_port_id, itrack);
                    module_manager.add_module_net_sink(top_module, net, under_sb_module_id,
                                                    under_sb_instance_id, under_sb_port_id, itrack);
                }

            }

    }

    void add_top_module_nets_connect_cb_and_cb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& cb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer, const t_rr_type& cb_type) {
    
            // VTR_LOG("Adding interlayer CB connections for CB[%lu][%lu][%lu] on layer %lu\n", rr_gsb.get_cb_x(cb_type), rr_gsb.get_cb_y(cb_type), layer, layer);

            /**
             * Goal of function is to connect the vertical tracks of the connection blocks in the top module
             * These tracks only exists if the inputs of grid locations are 3D
             * TODO: Make function work with more than 2 layers
             */

            /* Skip those Connection blocks that do not exist */
            if (false == rr_gsb.is_cb_exist(cb_type)) {
                return;
            }

            /* Skip if the cb does not contain any configuration bits! */
            if (true == connection_block_contain_only_routing_tracks(rr_gsb, cb_type)) {
                return;
            }

            RRChan cb_input_chan = rr_gsb.cb_input_chan(cb_type);

            size_t cb_input_chan_width = cb_input_chan.get_chan_width();

            if (cb_input_chan_width == 0){
                return;
            }

            /* We could have two different coordinators, one is the instance, the other is
            * the module */
            vtr::Point<size_t> instance_cb_coordinate(rr_gsb.get_cb_x(cb_type),
            rr_gsb.get_cb_y(cb_type));

            vtr::Point<size_t> module_gsb_coordinate(rr_gsb.get_x(), rr_gsb.get_y());
            size_t module_gsb_layer = layer;

            /* If we use compact routing hierarchy, we should find the unique module of
            * CB, which is added to the top module */
            if (true == compact_routing_hierarchy) {
                vtr::Point<size_t> gsb_coord(rr_gsb.get_x(), rr_gsb.get_y());
                const RRGSB& unique_mirror =
                device_rr_gsb.get_cb_unique_module(cb_type, gsb_coord, module_gsb_layer);
                module_gsb_coordinate.set_x(unique_mirror.get_x());
                module_gsb_coordinate.set_y(unique_mirror.get_y());
                module_gsb_layer = device_rr_gsb.get_cb_unique_module_layer(
                cb_type, device_rr_gsb.get_cb_unique_module_index(cb_type, gsb_coord,
                            module_gsb_layer));
            }

            /* This is the source cb that is added to the top module */
            const RRGSB& module_cb = device_rr_gsb.get_gsb(module_gsb_coordinate, module_gsb_layer);
            vtr::Point<size_t> module_cb_coordinate(module_cb.get_cb_x(cb_type),
                                                    module_cb.get_cb_y(cb_type));

            /* Collect source-related information */
            std::string cur_cb_module_name =
                generate_connection_block_module_name(cb_type, module_cb_coordinate, module_gsb_layer);
            ModuleId cur_cb_module = module_manager.find_module(cur_cb_module_name);

            VTR_ASSERT(true == module_manager.valid_module_id(cur_cb_module));
            /* Instance id should follow the instance cb coordinate */
            size_t cur_cb_instance =
              cb_instance_ids[layer][instance_cb_coordinate.x()][instance_cb_coordinate.y()];

            size_t sink_layer = 0;

            if (layer == 0) sink_layer = 1;
             
                
            /* We could have two different coordinators, one is the instance, the other is
            * the module */
           

            vtr::Point<size_t> sink_module_gsb_coordinate(rr_gsb.get_x(), rr_gsb.get_y());
            size_t sink_module_gsb_layer = sink_layer;

            /* If we use compact routing hierarchy, we should find the unique module of
            * CB, which is added to the top module */
            if (true == compact_routing_hierarchy) {
                vtr::Point<size_t> gsb_coord(rr_gsb.get_x(), rr_gsb.get_y());
                const RRGSB& unique_mirror =
                device_rr_gsb.get_cb_unique_module(cb_type, gsb_coord, sink_module_gsb_layer);
                sink_module_gsb_coordinate.set_x(unique_mirror.get_x());
                sink_module_gsb_coordinate.set_y(unique_mirror.get_y());
                sink_module_gsb_layer = device_rr_gsb.get_cb_unique_module_layer(
                cb_type, device_rr_gsb.get_cb_unique_module_index(cb_type, gsb_coord,
                            sink_module_gsb_layer));
            }

            /* This is the source cb that is added to the top module */
            const RRGSB& sink_module_cb = device_rr_gsb.get_gsb(sink_module_gsb_coordinate, sink_module_gsb_layer);
            vtr::Point<size_t> sink_module_cb_coordinate(sink_module_cb.get_cb_x(cb_type),
                                                    sink_module_cb.get_cb_y(cb_type));

            /* Collect source-related information */
            std::string sink_cb_module_name =
                generate_connection_block_module_name(cb_type, sink_module_cb_coordinate, sink_module_gsb_layer);
            ModuleId sink_cb_module = module_manager.find_module(sink_cb_module_name);

            VTR_ASSERT(true == module_manager.valid_module_id(sink_cb_module));
            /* Instance id should follow the instance cb coordinate */
            size_t sink_cb_instance =
                cb_instance_ids[sink_layer][instance_cb_coordinate.x()][instance_cb_coordinate.y()];
                
            std::string sink_port_name = generate_cb_interlayer_input_port_name(cb_type);

            std::string src_port_name = generate_cb_interlayer_output_port_name(cb_type);

            ModulePortId sink_port_id = module_manager.find_module_port(sink_cb_module, sink_port_name);

            VTR_ASSERT(true == module_manager.valid_module_port_id(sink_cb_module, sink_port_id));

            BasicPort sink_port = module_manager.module_port(sink_cb_module, sink_port_id);

            ModulePortId src_port_id = module_manager.find_module_port(cur_cb_module, src_port_name);

            VTR_ASSERT(true == module_manager.valid_module_port_id(cur_cb_module, src_port_id));

            BasicPort src_port = module_manager.module_port(cur_cb_module, src_port_id);

            if (sink_port.get_width() != src_port.get_width()){
                VTR_LOG("The source port %s in module %s has a different width than the sink port %s in module %s", src_port_name.c_str(), cur_cb_module_name.c_str(), sink_port_name.c_str(), sink_cb_module_name.c_str());
                VTR_LOG("Source port width: %lu, Sink port width: %lu", src_port.get_width(), sink_port.get_width());
            }

            // connect the source CB to the sink CB
            for (size_t itrack = 0; itrack < src_port.get_width(); ++itrack) {
                ModuleNetId net =
                    create_module_source_pin_net(module_manager, top_module, cur_cb_module,
                                                cur_cb_instance, src_port_id, itrack);

               

                module_manager.add_module_net_sink(top_module, net, sink_cb_module,
                                                sink_cb_instance, sink_port_id, itrack);
            }
    
            // ASSUMPTION BEING MADE:
            // 1. There are only 2 layers in the grid 
        }
}