/********************************************************************
 * This file includes functions to build bitstream from a mapped
 * FPGA fabric.
 * We decode the bitstream from configuration of routing multiplexers
 * and Look-Up Tables (LUTs) which locate in CLBs and global routing
 *architecture
 *******************************************************************/
#include <algorithm>
#include <vector>

/* Headers from vtrutil library */
#include "build_device_bitstream.h"
#include "build_grid_bitstream.h"
#include "build_routing_bitstream.h"
#include "memory_utils.h"
#include "module_manager_utils.h"
#include "openfpga_naming.h"
#include "vtr_assert.h"
#include "vtr_log.h"
#include "vtr_time.h"

/* begin namespace openfpga */
namespace openfpga {

/********************************************************************
 * Helper: true if a module's name marks it as a "layer" grouping module
 *******************************************************************/
static bool is_layer_module(const ModuleManager& module_manager,
                            const ModuleId& module) {
  return module_manager.module_name(module).find("layer") != std::string::npos;
}

/********************************************************************
 * Helper: true if a module has any configurable children.
 * The top-level module organizes its children per configuration region,
 * while every other module exposes them via the flat configurable_children
 * list -- this hides that distinction from callers.
 *******************************************************************/
static bool module_has_configurable_children(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ModuleId& parent_module) {
  if (parent_module == top_module) {
    for (const ConfigRegionId& config_region :
         module_manager.regions(parent_module)) {
      if (!module_manager
             .region_configurable_children(parent_module, config_region)
             .empty()) {
        return true;
      }
    }
    return false;
  }
  return 0 != module_manager.num_configurable_children(
                parent_module, ModuleManager::e_config_child_type::PHYSICAL);
}

/********************************************************************
 * Helper: return a module's configurable children, using the region-based
 * accessor for the top-level module and the flat accessor otherwise.
 *******************************************************************/
static std::vector<ModuleId> get_configurable_children(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ModuleId& parent_module) {
  std::vector<ModuleId> children;
  if (parent_module == top_module) {
    for (const ConfigRegionId& config_region :
         module_manager.regions(parent_module)) {
      for (const ModuleId& child_module :
           module_manager.region_configurable_children(parent_module,
                                                        config_region)) {
        children.push_back(child_module);
      }
    }
  } else {
    for (const ModuleId& child_module : module_manager.configurable_children(
           parent_module, ModuleManager::e_config_child_type::PHYSICAL)) {
      children.push_back(child_module);
    }
  }
  return children;
}

/********************************************************************
 * Estimate the number of blocks contributed by parent_module and
 * everything under it. parent_module is treated as its own "top" for
 * region lookups: the region-based accessor is only used when
 * parent_module == top_module, so this same function can be reused to
 * walk a layer module as if it were a top-level module in its own right.
 *******************************************************************/
static size_t rec_estimate_device_bitstream_num_blocks(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ModuleId& parent_module) {
  if (!module_has_configurable_children(module_manager, top_module,
                                        parent_module)) {
    return 0;
  }
  size_t sum = 0;
  for (const ModuleId& child_module :
       get_configurable_children(module_manager, top_module, parent_module)) {
    sum += rec_estimate_device_bitstream_num_blocks(module_manager,
                                                     top_module, child_module);
  }
  return sum + 1;
}

/********************************************************************
 * Sum the blocks contributed by a layer module's children WITHOUT
 * counting the layer module itself as a block. The real bitstream
 * builders never create a ConfigBlockId for the layer grouping -- they
 * add grid/SB/CB blocks flat, directly under top_block -- so the layer
 * module must not get a +1 of its own here.
 *******************************************************************/
static size_t sum_layer_module_children_num_blocks(
  const ModuleManager& module_manager, const ModuleId& layer_module) {
  if (!module_has_configurable_children(module_manager, layer_module,
                                        layer_module)) {
    return 0;
  }
  size_t sum = 0;
  for (const ModuleId& child_module :
       get_configurable_children(module_manager, layer_module, layer_module)) {
    sum += rec_estimate_device_bitstream_num_blocks(module_manager,
                                                     layer_module, child_module);
  }
  return sum;
}

/********************************************************************
 * Top-level entry point for block estimation.
 *
 * fpga_top itself may have no regions/configurable children registered
 * directly on it -- in the layered fabric case, its real configurable
 * content lives one level down, inside each fpga_layer_N module, which
 * is built as its own self-contained top-level-style module (with its
 * own regions). We detect that case and sum each layer module's
 * children directly into fpga_top's count, without counting the layer
 * module itself as a block.
 *******************************************************************/
static size_t estimate_device_bitstream_num_blocks_from_top(
  const ModuleManager& module_manager, const ModuleId& top_module) {
  if (module_has_configurable_children(module_manager, top_module,
                                       top_module)) {
    /* top_module has its own registered configurable children --
     * normal (non-layered) case */
    return rec_estimate_device_bitstream_num_blocks(module_manager,
                                                     top_module, top_module);
  }

  /* Layered case: top_module itself counts as 1 block, plus whatever
   * is found under each layer module's children (the layer module
   * itself is not counted as a block) */
  size_t sum = 1;
  for (const ModuleId& child_module : module_manager.child_modules(top_module)) {
    if (is_layer_module(module_manager, child_module)) {
      sum += sum_layer_module_children_num_blocks(module_manager, child_module);
    }
  }
  return sum;
}

/********************************************************************
 * Collect the predicted block names for diagnostics, mirroring the
 * counting logic above exactly. Only used when counts mismatch.
 *******************************************************************/
static void rec_collect_estimated_block_names(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ModuleId& parent_module, std::vector<std::string>& names) {
  if (!module_has_configurable_children(module_manager, top_module,
                                        parent_module)) {
    return;
  }
  for (const ModuleId& child_module :
       get_configurable_children(module_manager, top_module, parent_module)) {
    rec_collect_estimated_block_names(module_manager, top_module, child_module,
                                      names);
  }
  names.push_back(module_manager.module_name(parent_module));
}

static void collect_estimated_block_names_from_top(
  const ModuleManager& module_manager, const ModuleId& top_module,
  std::vector<std::string>& names) {
  if (module_has_configurable_children(module_manager, top_module,
                                       top_module)) {
    rec_collect_estimated_block_names(module_manager, top_module, top_module,
                                      names);
    return;
  }
  names.push_back(module_manager.module_name(top_module));
  for (const ModuleId& child_module : module_manager.child_modules(top_module)) {
    if (is_layer_module(module_manager, child_module)) {
      if (!module_has_configurable_children(module_manager, child_module,
                                            child_module)) {
        continue;
      }
      for (const ModuleId& grandchild :
           get_configurable_children(module_manager, child_module,
                                     child_module)) {
        rec_collect_estimated_block_names(module_manager, child_module,
                                          grandchild, names);
      }
    }
  }
}

/********************************************************************
 * Estimate the number of configuration bits to be added to the whole device
 * bitstream. Same top-vs-non-top structure as the blocks estimator above.
 *******************************************************************/
static size_t rec_estimate_device_bitstream_num_bits(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ModuleId& parent_module, const ConfigProtocol& config_protocol) {
  size_t num_bits = 0;

  if (!module_has_configurable_children(module_manager, top_module,
                                        parent_module)) {
    return 1;
  }

  if (parent_module == top_module) {
    for (const ConfigRegionId& config_region :
         module_manager.regions(parent_module)) {
      size_t curr_region_num_config_child =
        module_manager
          .region_configurable_children(parent_module, config_region)
          .size();
      size_t num_child_to_skip =
        estimate_num_configurable_children_to_skip_by_config_protocol(
          config_protocol, curr_region_num_config_child);
      curr_region_num_config_child -= num_child_to_skip;

      for (size_t ichild = 0; ichild < curr_region_num_config_child; ++ichild) {
        ModuleId child_module = module_manager.region_configurable_children(
          parent_module, config_region)[ichild];
        num_bits += rec_estimate_device_bitstream_num_bits(
          module_manager, top_module, child_module, config_protocol);
      }
    }
  } else {
    size_t num_configurable_children =
      module_manager
        .configurable_children(parent_module,
                               ModuleManager::e_config_child_type::PHYSICAL)
        .size();

    /* Frame-based configuration protocol will have 1 decoder
     * if there are more than 1 configurable children
     */
    if ((CONFIG_MEM_FRAME_BASED == config_protocol.type()) &&
        (2 <= num_configurable_children)) {
      num_configurable_children--;
    }

    for (size_t ichild = 0; ichild < num_configurable_children; ++ichild) {
      ModuleId child_module = module_manager.configurable_children(
        parent_module, ModuleManager::e_config_child_type::PHYSICAL)[ichild];
      num_bits += rec_estimate_device_bitstream_num_bits(
        module_manager, top_module, child_module, config_protocol);
    }
  }

  return num_bits;
}

static size_t sum_layer_module_children_num_bits(
  const ModuleManager& module_manager, const ModuleId& layer_module,
  const ConfigProtocol& config_protocol) {
  return rec_estimate_device_bitstream_num_bits(
    module_manager, layer_module, layer_module, config_protocol);
}

/********************************************************************
 * Top-level entry point for bit estimation. Mirrors
 * estimate_device_bitstream_num_blocks_from_top()'s layered-vs-normal
 * handling. Note bits are NOT affected by the layer-module double-count
 * issue (a module with no configurable children already contributes
 * exactly 1 bit via the leaf case), so each layer module's bit count is
 * simply summed in directly.
 *******************************************************************/
static size_t estimate_device_bitstream_num_bits_from_top(
  const ModuleManager& module_manager, const ModuleId& top_module,
  const ConfigProtocol& config_protocol) {
  if (module_has_configurable_children(module_manager, top_module,
                                       top_module)) {
    return rec_estimate_device_bitstream_num_bits(
      module_manager, top_module, top_module, config_protocol);
  }

  size_t sum = 0;
  for (const ModuleId& child_module : module_manager.child_modules(top_module)) {
    if (is_layer_module(module_manager, child_module)) {
      sum += sum_layer_module_children_num_bits(module_manager, child_module,
                                                 config_protocol);
    }
  }
  return sum;
}

/********************************************************************
 * A top-level function to build a bistream from the FPGA device
 * 1. It will organize the bitstream w.r.t. the hierarchy of module graphs
 *    describing the FPGA fabric
 * 2. It will decode configuration bits from routing multiplexers used in
 *    global routing architecture
 * 3. It will decode configuration bits from routing multiplexers and LUTs
 *    used in CLBs
 *
 * Note: this function create a bitstream which is binding to the module graphs
 * of the FPGA fabric that FPGA-X2P generates!
 * But it can be used to output a generic bitstream for VPR mapping FPGA
 *******************************************************************/
BitstreamManager build_device_bitstream(const VprContext& vpr_ctx,
                                        const OpenfpgaContext& openfpga_ctx,
                                        const bool& verbose) {
  std::string timer_message =
    std::string("\nBuild fabric-independent bitstream for implementation '") +
    vpr_ctx.atom().nlist.netlist_name() + std::string("'\n");
  vtr::ScopedStartFinishTimer timer(timer_message);

  /* Bitstream manager to be built */
  BitstreamManager bitstream_manager;

  /* Create the top-level block for bitstream
   * This is related to the top-level module of fpga
   */
  std::string top_block_name =
    openfpga_ctx.module_name_map().name(generate_fpga_top_module_name());
  ConfigBlockId top_block = bitstream_manager.add_block(top_block_name);
  ModuleId top_module = openfpga_ctx.module_graph().find_module(top_block_name);
  VTR_ASSERT(true == openfpga_ctx.module_graph().valid_module_id(top_module));

  /* Create the core block when the fpga_core is added */
  size_t num_blocks_to_reserve = 0;
  std::string core_block_name = generate_fpga_core_module_name();
  if (openfpga_ctx.module_name_map().name_exist(core_block_name)) {
    core_block_name = openfpga_ctx.module_name_map().name(core_block_name);
  }
  const ModuleId& core_module =
    openfpga_ctx.module_graph().find_module(core_block_name);
  if (openfpga_ctx.module_graph().valid_module_id(core_module)) {
    std::string core_inst_name =
      openfpga_ctx.module_graph().instance_name(top_module, core_module, 0);
    ConfigBlockId core_block = bitstream_manager.add_block(core_inst_name);
    bitstream_manager.add_child_block(top_block, core_block);
    /* Now we use the core_block as the top-level block for the remaining
     * functions */
    top_module = core_module;
    top_block = core_block;
    /* Count in fpga core as a block to reserve */
    num_blocks_to_reserve += 1;
  }

  /* Estimate the number of blocks to be added to the database */
  num_blocks_to_reserve += estimate_device_bitstream_num_blocks_from_top(
    openfpga_ctx.module_graph(), top_module);
  bitstream_manager.reserve_blocks(num_blocks_to_reserve);
  VTR_LOGV(verbose, "Reserved %lu configurable blocks\n",
           num_blocks_to_reserve);

  /* Estimate the number of bits to be added to the database */
  size_t num_bits_to_reserve = estimate_device_bitstream_num_bits_from_top(
    openfpga_ctx.module_graph(), top_module, openfpga_ctx.arch().config_protocol);
  bitstream_manager.reserve_bits(num_bits_to_reserve);
  VTR_LOGV(verbose, "Reserved %lu configuration bits\n", num_bits_to_reserve);

  /* Reserve child blocks for the top level block */
  bitstream_manager.reserve_child_blocks(
    top_block, count_module_manager_module_configurable_children(
                 openfpga_ctx.module_graph(), top_module,
                 ModuleManager::e_config_child_type::PHYSICAL));

  /* Create bitstream from grids */
  VTR_LOGV(verbose, "Building grid bitstream...\n");
  build_grid_bitstream(
    bitstream_manager, top_block, openfpga_ctx.module_graph(),
    openfpga_ctx.module_name_map(), openfpga_ctx.fabric_tile(),
    openfpga_ctx.arch().circuit_lib, openfpga_ctx.mux_lib(),
    vpr_ctx.device().grid, 0, vpr_ctx.atom(),
    openfpga_ctx.vpr_device_annotation(),
    openfpga_ctx.vpr_clustering_annotation(),
    openfpga_ctx.vpr_placement_annotation(),
    openfpga_ctx.vpr_bitstream_annotation(), verbose);
  VTR_LOGV(verbose, "Done\n");

  /* Create bitstream from routing architectures */
  VTR_LOGV(verbose, "Building routing bitstream...\n");
  build_routing_bitstream(
    bitstream_manager, top_block, openfpga_ctx.module_graph(),
    openfpga_ctx.module_name_map(), openfpga_ctx.fabric_tile(),
    openfpga_ctx.arch().circuit_lib, openfpga_ctx.mux_lib(), vpr_ctx.atom(),
    openfpga_ctx.vpr_device_annotation(), openfpga_ctx.vpr_routing_annotation(),
    vpr_ctx.device().rr_graph, openfpga_ctx.device_rr_gsb(),
    openfpga_ctx.flow_manager().compress_routing(), verbose);

  VTR_LOGV(verbose, "Done\n");

  VTR_LOGV(verbose, "Decoded %lu configuration bits into %lu blocks\n",
           bitstream_manager.num_bits(), bitstream_manager.num_blocks());

  /* Diagnostic: only runs if the counts actually mismatch, so it costs
   * nothing when everything is correct. If this still fires, it prints
   * the exact block names that differ between prediction and reality. */
  if (num_blocks_to_reserve != bitstream_manager.num_blocks()) {
    std::vector<std::string> estimated_names;
    collect_estimated_block_names_from_top(openfpga_ctx.module_graph(),
                                           top_module, estimated_names);
    estimated_names.push_back(bitstream_manager.block_name(top_block));

    std::vector<std::string> actual_names;
    for (const ConfigBlockId& blk : bitstream_manager.blocks()) {
      actual_names.push_back(bitstream_manager.block_name(blk));
    }

    std::sort(estimated_names.begin(), estimated_names.end());
    std::sort(actual_names.begin(), actual_names.end());

    std::vector<std::string> only_in_estimate, only_in_actual;
    std::set_difference(estimated_names.begin(), estimated_names.end(),
                        actual_names.begin(), actual_names.end(),
                        std::back_inserter(only_in_estimate));
    std::set_difference(actual_names.begin(), actual_names.end(),
                        estimated_names.begin(), estimated_names.end(),
                        std::back_inserter(only_in_actual));

    VTR_LOG_ERROR("Block count mismatch: estimated=%zu actual=%zu\n",
                  num_blocks_to_reserve, bitstream_manager.num_blocks());
    VTR_LOG("---- Predicted but NOT actually built (%zu) ----\n",
            only_in_estimate.size());
    for (const std::string& n : only_in_estimate) {
      VTR_LOG("  estimator-only: %s\n", n.c_str());
    }
    VTR_LOG("---- Actually built but NOT predicted (%zu) ----\n",
            only_in_actual.size());
    for (const std::string& n : only_in_actual) {
      VTR_LOG("  actual-only: %s\n", n.c_str());
    }
  }

  VTR_ASSERT(num_blocks_to_reserve == bitstream_manager.num_blocks());
  VTR_ASSERT(num_bits_to_reserve == bitstream_manager.num_bits());

  return bitstream_manager;
}

} /* end namespace openfpga */