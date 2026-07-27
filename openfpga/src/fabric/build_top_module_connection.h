#ifndef BUILD_TOP_MODULE_CONNECTION_H
#define BUILD_TOP_MODULE_CONNECTION_H

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

/********************************************************************
 * Function declaration
 *******************************************************************/

/* begin namespace openfpga */
namespace openfpga {

void add_top_module_nets_connect_grids_and_gsbs(
  ModuleManager& module_manager, const ModuleId& top_module,
  const VprDeviceAnnotation& vpr_device_annotation, const DeviceGrid& grids,
  const vtr::NdMatrix<size_t, 3>& grid_instance_ids,
  const RRGraphView& rr_graph, const DeviceRRGSB& device_rr_gsb,
  const vtr::NdMatrix<size_t, 3>& sb_instance_ids,
  const std::map<t_rr_type, vtr::NdMatrix<size_t, 3>>& cb_instance_ids,
  const bool& compact_routing_hierarchy, const bool& duplicate_grid_pin, const size_t& layer);

int add_top_module_global_ports_from_grid_modules(
  ModuleManager& module_manager, const ModuleId& top_module,
  const TileAnnotation& tile_annotation,
  const VprDeviceAnnotation& vpr_device_annotation, const DeviceGrid& grids,
  const RRGraphView& rr_graph,
  const DeviceRRGSB& device_rr_gsb,
  const std::map<t_rr_type, vtr::NdMatrix<size_t, 3>>& cb_instance_ids,
  const vtr::NdMatrix<size_t, 3>& grid_instance_ids, const ClockNetwork& clk_ntwk,
  const RRClockSpatialLookup& rr_clock_lookup, const bool& perimeter_cb, const size_t& layer);

void add_top_module_nets_prog_clock(ModuleManager& module_manager,
                                    const ModuleId& top_module,
                                    const ModulePortId& src_port,
                                    const ConfigProtocol& config_protocol);

} /* end namespace openfpga */

#endif
