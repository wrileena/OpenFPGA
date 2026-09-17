/**
 * Header file for the vertical connection of the top-level module
 * The function declarations in this file are modified versions of some of the ones in build_top_module_connection.h
 */

#ifndef BUILD_TOP_MODULE_VERTICAL_CONNECTION_H
#define BUILD_TOP_MODULE_VERTICAL_CONNECTION_H

/********************************************************************
 * Include header files that are required by function declaration
 *******************************************************************/
#include <vector>

#include "clock_network.h"
#include "config_protocol.h"
#include "device_grid.h"
#include "device_rr_gsb.h"
#include "module_manager.h"
#include "rr_clock_spatial_lookup.h"
#include "rr_graph_view.h"
#include "tile_annotation.h"
#include "vpr_device_annotation.h"
#include "vtr_geometry.h"
#include "vtr_ndmatrix.h"

namespace openfpga{
    void add_top_module_nets_connect_sb_and_sb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer);

    void add_top_module_nets_connect_cb_and_cb(
        ModuleManager& module_manager, const ModuleId& top_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const RRGSB& rr_gsb, const vtr::NdMatrix<size_t, 3>& cb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer, const t_rr_type& cb_type);
     /* Exposes SB/CB interlayer signals as boundary ports on a layer_module.
      * Call once per layer, from inside that layer's own build, while
      * sb_instance_ids/cb_instance_ids for that layer are still in scope. */
    void add_layer_module_interlayer_ports(
        ModuleManager& module_manager, const ModuleId& layer_module,
        const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
        const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
        const std::map<t_rr_type, vtr::NdMatrix<size_t, 3>>& cb_instance_ids,
        const bool& compact_routing_hierarchy, const size_t& layer);

        /* Connects fpga_layer_i <-> fpga_layer_{i+1} at the true top module.
        * Call only once, after ALL layer modules have been fully built. */
    void add_top_module_nets_connect_layer_and_layer(
        ModuleManager& module_manager, const ModuleId& top_module,
        const DeviceRRGSB& device_rr_gsb,
        const std::vector<ModuleId>& layer_module_ids,
        const std::vector<size_t>& layer_instance_ids,
        const size_t& num_layers);

}

#endif