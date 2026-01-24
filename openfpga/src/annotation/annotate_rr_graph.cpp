/********************************************************************
 * This file includes functions that are used to annotate device-level
 * information, in particular the routing resource graph
 *******************************************************************/
/* Headers from vtrutil library */
#include "vtr_assert.h"
#include "vtr_log.h"
#include "vtr_time.h"

/* Headers from openfpgautil library */
#include "openfpga_side_manager.h"

/* Headers from vpr library */
#include "annotate_rr_graph.h"
#include "openfpga_rr_graph_utils.h"
#include "physical_types.h"
#include "rr_graph_view_util.h"

/* begin namespace openfpga */
namespace openfpga {

// Function to get the source node for a vertical channel node
RRNodeId get_src_node(const RRGraphView& rr_graph, RRNodeId node_id){
  for (auto edge : rr_graph.node_in_edges(node_id)){
    if (rr_graph.edge_sink_node((RREdgeId) edge) == node_id) return rr_graph.edge_src_node((RREdgeId) edge);
  }
  return RRNodeId::INVALID();
}

// Function to get the sink node for a vertical channel node
RRNodeId get_sink_node(const RRGraphView& rr_graph, RRNodeId node_id){
  for (auto edge : rr_graph.edge_range(node_id)){
    if (rr_graph.edge_src_node((RREdgeId) edge) == node_id) return rr_graph.edge_sink_node((RREdgeId) edge);
  }
  return RRNodeId::INVALID();
}

/* Build a RRChan Object with the given channel type and coorindators */
static RRChan build_one_rr_chan(const DeviceContext& vpr_device_ctx,
                                const t_rr_type& chan_type, const size_t& layer,
                                vtr::Point<size_t>& chan_coord, const vtr::Point<size_t>& sb_coord) {
  std::vector<RRNodeId> chan_rr_nodes;

  /* Create a rr_chan object and check if it is unique in the graph */
  RRChan rr_chan;
  /* Fill the information */
  rr_chan.set_type(chan_type);

  /* Collect rr_nodes for this channel */
  chan_rr_nodes = find_rr_graph_chan_nodes(
    vpr_device_ctx.rr_graph, layer, chan_coord.x(), chan_coord.y(), chan_type);
  /* Fill the rr_chan */
  for (const RRNodeId& chan_rr_node : chan_rr_nodes) {
    
    // do not include vertical nodes in channel
    if (vpr_device_ctx.rr_graph.node_direction(chan_rr_node) == Direction::ABOVE_DEC || vpr_device_ctx.rr_graph.node_direction(chan_rr_node) == Direction::UNDER_INC ||
        vpr_device_ctx.rr_graph.node_direction(chan_rr_node) == Direction::ABOVE_INC || vpr_device_ctx.rr_graph.node_direction(chan_rr_node) == Direction::UNDER_DEC){
      continue;
    }

    rr_chan.add_node(vpr_device_ctx.rr_graph, chan_rr_node,
                     vpr_device_ctx.rr_graph.node_segment(chan_rr_node));
  }

  return rr_chan;
}

/* Build a RRChan Object with the given channel type and coorindators */
/**
 * Directions explanation:
 * ABOVE_DEC: input to layer, Port is on the above side of SB and the channel is decreasing in Z axis so it's going towards SB AKA input
 * ABOVE_INC: output of layer, Port is on the above side of SB and the channel is increasing in Z axis so it's going away from SB AKA output
 * UNDER_DEC: output of layer, Port is on the under side of SB and the channel is decreasing in Z axis so it's going away from SB AKA output
 * UNDER_INC: input to layer, Port is on the under side of SB and the channel is increasing in Z axis so it's going towards SB AKA input
 */
static RRChan build_one_interlayer_rr_chan(const DeviceContext& vpr_device_ctx,
                                const size_t& layer, const vtr::Point<size_t>& chan_coord, 
                                e_side side, const vtr::Point<size_t>& sb_coord) {
  std::vector<RRNodeId> chan_rr_nodes;

  /* Create a rr_chan object and check if it is unique in the graph */
  RRChan rr_chan;
  /* Fill the information */
  rr_chan.set_type(CHANX);

  std::vector<RRNodeId> input_rr_nodes;
  std::vector<RRNodeId> output_rr_nodes;

  //get both CHANX and CHANY nodes since vertically there's no difference between them
  for (size_t type = 0; type < 2; type++){
    t_rr_type cur_type;
    if(type == 0) cur_type = CHANX;
    else cur_type = CHANY;
    // Get all vertical channels from each side. And group them together into 1 channel
    for (size_t cur_side = 0; cur_side < 4; cur_side++){/* 0 = TOP, 1 = RIGHT, 2 = BOTTOM, 3 = LEFT*/
      vtr::Point<size_t> cur_coord;
      cur_coord.set_x(chan_coord.x());
      cur_coord.set_y(chan_coord.y());
      if (cur_side == 0){
        cur_coord.set_y(chan_coord.y() + 1);
      }
      else if (cur_side == 1){
        cur_coord.set_x(chan_coord.x() + 1);
      }

      chan_rr_nodes = find_rr_graph_chan_nodes(
        vpr_device_ctx.rr_graph, layer, cur_coord.x(), cur_coord.y(), cur_type);

      /* Fill the rr_chan */
      for (const RRNodeId& chan_rr_node : chan_rr_nodes) {
        // only want vertical nodes

        Direction chan_node_direction = vpr_device_ctx.rr_graph.node_direction(chan_rr_node);

        // Only care about vertical channels (ABOVE INC, ABOVE DEC, UNDER INC, UNDER DEC)
        if (chan_node_direction == Direction::INC || chan_node_direction == Direction::DEC){
          continue;
        }

        short chan_node_xlow = vpr_device_ctx.rr_graph.node_xlow(chan_rr_node);
        short chan_node_ylow = vpr_device_ctx.rr_graph.node_ylow(chan_rr_node);

        // Interlayer channels only exist if the sb coords are the same as its coords
        if (chan_node_xlow != sb_coord.x() || chan_node_ylow != sb_coord.y()){
          continue;
        }

        if (side == e_side::UNDER){
          // input of layer
          if (chan_node_direction == Direction::UNDER_INC){ 
            RRNodeId sink_node = get_sink_node(vpr_device_ctx.rr_graph, chan_rr_node);
            VTR_ASSERT(sink_node != RRNodeId::INVALID());

            short node_xlow = vpr_device_ctx.rr_graph.node_xlow(sink_node);
            short node_xhigh = vpr_device_ctx.rr_graph.node_xhigh(sink_node);
            short node_ylow = vpr_device_ctx.rr_graph.node_ylow(sink_node);
            short node_yhigh = vpr_device_ctx.rr_graph.node_yhigh(sink_node);

            Direction node_direction = vpr_device_ctx.rr_graph.node_direction(sink_node);
            t_rr_type node_type = vpr_device_ctx.rr_graph.node_type(sink_node);

            // if x and y coords are same as sb and the chan is inc then its an input to the SB
            if (node_xhigh == sb_coord.x() && node_yhigh == sb_coord.y() && node_direction == Direction::DEC){
                // make sure the node is not already in the vector, we don't want repeat nodes
                if (std::find(input_rr_nodes.begin(), input_rr_nodes.end(), chan_rr_node) == input_rr_nodes.end()) input_rr_nodes.push_back(chan_rr_node);
            }

            // if the x or y coord is off by 1 and the chan is dec then its an input to the SB
            else if ((node_xlow == sb_coord.x() + 1 && node_ylow == sb_coord.y() && node_type == CHANX && node_direction == Direction::INC) ||
                     (node_xlow == sb_coord.x() && node_ylow == sb_coord.y() + 1 && node_type == CHANY && node_direction == Direction::INC)){
              // make sure the node is not already in the vector, we don't want repeat nodes
              if (std::find(input_rr_nodes.begin(), input_rr_nodes.end(), chan_rr_node) == input_rr_nodes.end()) input_rr_nodes.push_back(chan_rr_node);
            }
          }
          
          // output of layer 
          else if (chan_node_direction == Direction::UNDER_DEC){
            RRNodeId source_node = get_src_node(vpr_device_ctx.rr_graph, chan_rr_node);
            VTR_ASSERT(source_node != RRNodeId::INVALID());

            short node_xlow = vpr_device_ctx.rr_graph.node_xlow(source_node);
            short node_xhigh = vpr_device_ctx.rr_graph.node_xhigh(source_node);
            short node_ylow = vpr_device_ctx.rr_graph.node_ylow(source_node);
            short node_yhigh = vpr_device_ctx.rr_graph.node_yhigh(source_node);

            Direction node_direction = vpr_device_ctx.rr_graph.node_direction(source_node);
            t_rr_type node_type = vpr_device_ctx.rr_graph.node_type(source_node);


            // if x and y coords are same as sb and the chan is dec then its an output of the SB
            if (node_xhigh >= sb_coord.x() && node_yhigh >= sb_coord.y() && node_direction == Direction::INC){
                // make sure the node is not already in the vector, we don't want repeat nodes
                if (std::find(output_rr_nodes.begin(), output_rr_nodes.end(), chan_rr_node) == output_rr_nodes.end()) output_rr_nodes.push_back(chan_rr_node);
            }
            // if the x or y coord is off by 1 and the chan is inc then its an output of the SB
            else if ((node_xlow <= sb_coord.x() + 1 && node_ylow == sb_coord.y() && node_type == CHANX && node_direction == Direction::DEC) ||
                     (node_xlow == sb_coord.x() && node_ylow <= sb_coord.y() + 1 && node_type == CHANY && node_direction == Direction::DEC)){
              // make sure the node is not already in the vector, we don't want repeat nodes
              if (std::find(output_rr_nodes.begin(), output_rr_nodes.end(), chan_rr_node) == output_rr_nodes.end()) output_rr_nodes.push_back(chan_rr_node);
            }
          }
          
        }
        
        else{
          VTR_ASSERT(side == e_side::ABOVE);
        
          // input to layer, so the relevant node to determine location is this node's sink
          if (chan_node_direction == Direction::ABOVE_DEC){ 
            RRNodeId sink_node = get_sink_node(vpr_device_ctx.rr_graph, chan_rr_node);
            VTR_ASSERT(sink_node != RRNodeId::INVALID());

            short node_xlow = vpr_device_ctx.rr_graph.node_xlow(sink_node);
            short node_xhigh = vpr_device_ctx.rr_graph.node_xhigh(sink_node);
            short node_ylow = vpr_device_ctx.rr_graph.node_ylow(sink_node);
            short node_yhigh = vpr_device_ctx.rr_graph.node_yhigh(sink_node);

            Direction node_direction = vpr_device_ctx.rr_graph.node_direction(sink_node);
            t_rr_type node_type = vpr_device_ctx.rr_graph.node_type(sink_node);

            // if x and y coords are same as sb and the chan is inc then its an input to the SB
            if (node_xhigh == sb_coord.x() && node_yhigh == sb_coord.y() && node_direction == Direction::DEC){
                // make sure the node is not already in the vector, we don't want repeat nodes
                if (std::find(input_rr_nodes.begin(), input_rr_nodes.end(), chan_rr_node) == input_rr_nodes.end()) input_rr_nodes.push_back(chan_rr_node);
            }
            // if the x or y coord is off by 1 and the chan is dec then its an input to the SB
            else if ((node_xlow == sb_coord.x() + 1 && node_ylow == sb_coord.y() && node_type == CHANX && node_direction == Direction::INC) ||
                     (node_xlow == sb_coord.x() && node_ylow == sb_coord.y() + 1 && node_type == CHANY && node_direction == Direction::INC)){
              // make sure the node is not already in the vector, we don't want repeat nodes
              if (std::find(input_rr_nodes.begin(), input_rr_nodes.end(), chan_rr_node) == input_rr_nodes.end()) input_rr_nodes.push_back(chan_rr_node);
            }
          }

          // output of layer, so the relevant node to determine location is this node's source
          else if (chan_node_direction == Direction::ABOVE_INC){
            RRNodeId source_node = get_src_node(vpr_device_ctx.rr_graph, chan_rr_node);
            VTR_ASSERT(source_node != RRNodeId::INVALID());

            short node_xlow = vpr_device_ctx.rr_graph.node_xlow(source_node);
            short node_xhigh = vpr_device_ctx.rr_graph.node_xhigh(source_node);
            short node_ylow = vpr_device_ctx.rr_graph.node_ylow(source_node);
            short node_yhigh = vpr_device_ctx.rr_graph.node_yhigh(source_node);

            Direction node_direction = vpr_device_ctx.rr_graph.node_direction(source_node);
            t_rr_type node_type = vpr_device_ctx.rr_graph.node_type(source_node);

            // if x and y coords are same as sb and the chan is dec then its an output of the SB
            if (node_xhigh >= sb_coord.x() && node_yhigh >= sb_coord.y() && node_direction == Direction::INC){
              // make sure the node is not already in the vector, we don't want repeat nodes
              if (std::find(output_rr_nodes.begin(), output_rr_nodes.end(), chan_rr_node) == output_rr_nodes.end()) output_rr_nodes.push_back(chan_rr_node);
            }
            // if the x or y coord is off by 1 and the chan is inc then its an output of the SB
            else if ((node_xlow <= sb_coord.x() + 1 && node_ylow == sb_coord.y() && node_type == CHANX && node_direction == Direction::DEC) ||
                     (node_xlow == sb_coord.x() && node_ylow <= sb_coord.y() + 1 && node_type == CHANY && node_direction == Direction::DEC)){
              // make sure the node is not already in the vector, we don't want repeat nodes
              if (std::find(output_rr_nodes.begin(), output_rr_nodes.end(), chan_rr_node) == output_rr_nodes.end()) output_rr_nodes.push_back(chan_rr_node);
            }
          }
          
        }

       
        

      }
    }
  }
   // Alternate adding nodes to rr_chan going output_rr_node then input then output, etc.
   // This is needed because the Fabric Builder expects the nodes to be in this order for the channel
   // to be built correctly
  for (size_t i = 0; i < output_rr_nodes.size(); i++){
    rr_chan.add_node(vpr_device_ctx.rr_graph, output_rr_nodes[i],
                    vpr_device_ctx.rr_graph.node_segment(output_rr_nodes[i]));
    if (i < input_rr_nodes.size()){
      rr_chan.add_node(vpr_device_ctx.rr_graph, input_rr_nodes[i],
                    vpr_device_ctx.rr_graph.node_segment(input_rr_nodes[i]));
    }
  }
  output_rr_nodes.clear();
  input_rr_nodes.clear();
  return rr_chan;
}

static RRChan build_chan_from_nodes(const DeviceContext& vpr_device_ctx, const std::vector<RRNodeId>& rr_nodes, const t_rr_type& chan_type){
  RRChan rr_chan;
  rr_chan.set_type(chan_type);
  for (const RRNodeId& chan_rr_node : rr_nodes) {
    rr_chan.add_node(vpr_device_ctx.rr_graph, chan_rr_node,
                     vpr_device_ctx.rr_graph.node_segment(chan_rr_node));
  }
  return rr_chan;
}

/**
 * Function to check if a node is connected to a layer, to determine whether to include as a part of the SB or not
 */
bool is_node_connected_to_layer(const RRGraphView& rr_graph, RRNodeId& node_to_check, const size_t& layer, const t_rr_type& rr_type){

  // Loop over the out edges of the node, if any of the sinks are not on the same layer, return true
  // else false
  if (rr_type == t_rr_type::OPIN){

    for (t_edge_size edge_out : rr_graph.node_out_edges(node_to_check)){
      RRNodeId sink_node = rr_graph.edge_sink_node(node_to_check, edge_out);    

      //sanity check
      VTR_ASSERT(sink_node != RRNodeId::INVALID());
      VTR_ASSERT(sink_node != node_to_check);

      if (rr_graph.node_layer(sink_node) != layer){
        return true;
      }
    }
  }
  if (rr_type == t_rr_type::IPIN){
    for (RREdgeId edge_in: rr_graph.node_in_edges(node_to_check)){
      RRNodeId source_node = rr_graph.edge_src_node(edge_in);

      //sanity check
      VTR_ASSERT(source_node != RRNodeId::INVALID());
      VTR_ASSERT(source_node != node_to_check);

      if (rr_graph.node_layer(source_node) != layer){
        return true;
      }
    }
  }

  return false;

}

/** 
 * Helper function to get all opin nodes on other layers for 3D CB option
 * TODO: Only get OPIN nodes that are connected to this channel, not all OPIN nodes 
*/
std::vector<RRNodeId> find_interlayer_rr_graph_grid_nodes(const RRGraphView& rr_graph,
                                               const DeviceGrid& device_grid,
                                               const size_t& layer,
                                               const int& x,
                                               const int& y,
                                               const t_rr_type& rr_type,
                                               const e_side& side,
                                               bool include_clock=false) {

    size_t num_layers = device_grid.get_num_layers();
    std::vector<RRNodeId> indices;

    VTR_ASSERT(rr_type == IPIN || rr_type == OPIN);

    /* Ensure that (x, y) is a valid location in grids */
    if (size_t(x) > device_grid.width() - 1 || size_t(y) > device_grid.height() - 1) {
      return indices;
    }

    /* Ensure we have a valid side */
    VTR_ASSERT(side != NUM_2D_SIDES);
    for (size_t ilayer = 0; ilayer < num_layers; ++ilayer) {
      if (ilayer == layer) {
        continue;
      }
      /* Find all the pins on the side of the grid */
      t_physical_tile_loc tile_loc(x, y, ilayer);
      int width_offset = device_grid.get_width_offset(tile_loc);
      int height_offset = device_grid.get_height_offset(tile_loc);
      
      for (int pin = 0; pin < device_grid.get_physical_type(tile_loc)->num_pins; ++pin) {
          /* Skip those pins have been ignored during rr_graph build-up */
          if (true == device_grid.get_physical_type(tile_loc)->is_ignored_pin[pin]) {
              /* If specified, force to include all the clock pins */
              if (!include_clock || std::find(device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().begin(), device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().end(), pin) == device_grid.get_physical_type(tile_loc)->get_clock_pins_indices().end()) {
                  continue;
              }
          }
          
          if (false == device_grid.get_physical_type(tile_loc)->pinloc[width_offset][height_offset][side][pin]) {
              /* Not the pin on this side, we skip */
              continue;
          }

          /* Try to find the rr node */
          RRNodeId rr_node_index = rr_graph.node_lookup().find_node(ilayer, x, y, rr_type, pin, side);
          if (rr_node_index != RRNodeId::INVALID()) {
              // need to check if this node has any edges connected to this layer if not don't add it
              if (is_node_connected_to_layer(rr_graph, rr_node_index, layer, rr_type)){
                indices.push_back(rr_node_index);
              }
          }
      }
    }

    return indices;
}

/** 
 * Helper function to get all channel nodes on other layers connected to IPINs on this layer for 3D CB option
 */ 
std::vector<RRNodeId> find_interlayer_chans_connected_to_input(const RRGraphView& rr_graph, const DeviceGrid& device_grid, const size_t& layer, const int&x, const int& y, const t_rr_type& rr_type, const e_side& side){
  size_t num_layers = device_grid.get_num_layers();
  std::vector<RRNodeId> indices;

  VTR_ASSERT(rr_type == IPIN);

  /* Ensure that (x, y) is a valid location in grids */
  if (size_t(x) > device_grid.width() - 1 || size_t(y) > device_grid.height() - 1) {
    return indices;
  }

  t_physical_tile_loc tile_loc(x, y, layer);
  int width_offset = device_grid.get_width_offset(tile_loc);
  int height_offset = device_grid.get_height_offset(tile_loc);

  /* Ensure we have a valid side */
  VTR_ASSERT(side != NUM_2D_SIDES);

  for (int pin = 0; pin < device_grid.get_physical_type(tile_loc)->num_pins; ++pin) {
      /* Skip those pins have been ignored during rr_graph build-up */
      if (true == device_grid.get_physical_type(tile_loc)->is_ignored_pin[pin]) {
          continue;
      }
      
      if (false == device_grid.get_physical_type(tile_loc)->pinloc[width_offset][height_offset][side][pin]) {
          /* Not the pin on this side, we skip */
          continue;
      }

      /* Get RR Node and find the channels connected to it that are interlayer */
      RRNodeId rr_node_index = rr_graph.node_lookup().find_node(layer, x, y, rr_type, pin, side);
      if (rr_node_index != RRNodeId::INVALID()) {
          // check the input edges of this node and figure out which ones are not on the same layer
          for (RREdgeId edge_in: rr_graph.node_in_edges(rr_node_index)){
            RRNodeId source_node = rr_graph.edge_src_node(edge_in);
            
            // To account for direct connections
            if (rr_graph.node_type(source_node) != CHANX && rr_graph.node_type(source_node) != CHANY){
              continue;
            }

            if (rr_graph.node_layer(source_node) != layer && indices.end() == std::find(indices.begin(), indices.end(), source_node)){
              indices.push_back(source_node);
            }
          }
      }
  }


  return indices;
}

/** 
 * Helper function to get all channel nodes on this layer that are connected to IPINs on other layers for 3D CB option
 */ 
std::vector<RRNodeId> find_interlayer_nodes_connected_to_other_layers(const RRGraphView& rr_graph, const DeviceGrid& device_grid, const size_t& layer, const RRChan& rr_chan, size_t x, size_t y){

  std::vector<RRNodeId> indices;

  for (size_t itrack = 0; itrack < rr_chan.get_chan_width(); ++itrack){
    RRNodeId rr_node = rr_chan.get_node(itrack);
    
    for (auto& edge: rr_graph.node_out_edges(rr_node)){
      RRNodeId sink_node = rr_graph.edge_sink_node(rr_node, edge);

      if (rr_graph.node_type(sink_node) != IPIN) continue;

      e_side node_side = NUM_2D_SIDES;

      for (const e_side& side : TOTAL_2D_SIDES){
        if (rr_graph.is_node_on_specific_side(sink_node, side)){
          node_side = side;
          break;
        }
      }

      // sanity check
      VTR_ASSERT(node_side != NUM_2D_SIDES);

      if (node_side == e_side::TOP || node_side == e_side::RIGHT){
        if (rr_graph.node_xlow(sink_node) != x || rr_graph.node_ylow(sink_node) != y){
          continue;
        }
      }

      if (node_side == e_side::BOTTOM){
        if (rr_graph.node_xlow(sink_node) != x || rr_graph.node_ylow(sink_node) != y + 1){
          continue;
        }
      }
      if (node_side == e_side::LEFT){
        if (rr_graph.node_xlow(sink_node) != x + 1 || rr_graph.node_ylow(sink_node) != y){
          continue;
        }
      }

      if (rr_graph.node_layer(sink_node) != layer && indices.end() == std::find(indices.begin(), indices.end(), rr_node)){
        indices.push_back(rr_node);
      }
    }
  }

  return indices;
}



/* Build a General Switch Block (GSB)
 * which includes:
 * [I] A Switch Box subckt consists of following ports:
 * 1. Channel Y [x][y] inputs
 * 2. Channel X [x+1][y] inputs
 * 3. Channel Y [x][y-1] outputs
 * 4. Channel X [x][y] outputs
 * 5. Grid[x][y+1] Right side outputs pins
 * 6. Grid[x+1][y+1] Left side output pins
 * 7. Grid[x+1][y+1] Bottom side output pins
 * 8. Grid[x+1][y] Top side output pins
 * 9. Grid[x+1][y] Left side output pins
 * 10. Grid[x][y] Right side output pins
 * 11. Grid[x][y] Top side output pins
 * 12. Grid[x][y+1] Bottom side output pins
 *
 *    --------------          --------------
 *    |            |   CBY    |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |  [x][y+1]  | [x][y+1] | [x+1][y+1] |
 *    |            |          |            |
 *    --------------          --------------
 *                  ----------
 *     ChanX & CBX  | Switch |     ChanX
 *       [x][y]     |   Box  |    [x+1][y]
 *                  | [x][y] |
 *                  ----------
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |   [x][y]   |  [x][y]  |  [x+1][y]  |
 *    |            |          |            |
 *    --------------          --------------
 * For channels chanY with INC_DIRECTION on the top side, they should be marked as outputs 
 * For channels chanY with DEC_DIRECTION on the top side, they should be marked as inputs 
 * For channels chanY with INC_DIRECTION on the bottom side, they should be marked as inputs 
 * For channels chanY with DEC_DIRECTION on the bottom side, they should be marked as outputs 
 * For channels chanX with INC_DIRECTION on the left side, they should be marked as inputs 
 * For channels chanX with DEC_DIRECTION on the left side, they should be marked as outputs
 * For channels chanX with INC_DIRECTION on the right side, they should be marked as outputs 
 * For channels chanX with DEC_DIRECTION on the right side, they should be marked as inputs
 *
 * [II] A X-direction Connection Block [x][y]
 * The connection block shares the same routing channel[x][y] with the Switch Block 
 * We just need to fill the ipin nodes at TOP and BOTTOM sides 
 * as well as properly fill the ipin_grid_side information 
 * 
 * [III] A Y-direction Connection Block [x][y+1] 
 * The connection block shares the same routing channel[x][y+1] with the Switch Block 
 * We just need to fill the ipin nodes at LEFT and RIGHT sides 
 * as well as properly fill the ipin_grid_side information
 */
static RRGSB build_rr_gsb(const DeviceContext& vpr_device_ctx,
                          const vtr::Point<size_t>& gsb_range,
                          const size_t& layer,
                          const vtr::Point<size_t>& gsb_coord,
                          const bool& perimeter_cb, const bool& include_clock) {
  /* Create an object to return */
  RRGSB rr_gsb;

  VTR_ASSERT(gsb_coord.x() <= gsb_range.x());
  VTR_ASSERT(gsb_coord.y() <= gsb_range.y());

  /* Coordinator initialization */
  rr_gsb.set_coordinate(gsb_coord.x(), gsb_coord.y());

  /* Basic information*/
  rr_gsb.init_num_sides(4); /* Fixed number of sides */

  std::vector<RRNodeId> interlayer_cb_output_x;
  std::vector<RRNodeId> interlayer_cb_output_y;

  /* Find all rr_nodes of channels */
  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);
    vtr::Point<size_t> coordinate =
      rr_gsb.get_side_block_coordinate(side_manager.get_side());
    RRChan rr_chan;
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes(2);
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
    std::vector<RRNodeId> temp_cb_output_nodes;
    enum e_side opin_grid_side[2] = {NUM_2D_SIDES, NUM_2D_SIDES};
    enum PORTS chan_dir_to_port_dir_mapping[2] = {
      OUT_PORT, IN_PORT}; /* 0: INC_DIRECTION => ?; 1: DEC_DIRECTION => ? */

    switch (side) {
      case TOP: /* TOP = 0 */
        if (gsb_coord.y() == gsb_range.y()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x][y+1] RIGHT side outputs pins */
        opin_grid_side[0] = RIGHT;
        /* Grid[x+1][y+1] left side outputs pins */
        opin_grid_side[1] = LEFT;
        /* Include Grid[x][y+1] RIGHT side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* Include Grid[x+1][y+1] Left side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[1]);


        // Check if there are any interlayer connections needed for 3D CB, need to check if these nodes are connected though since if the CB is 2D that is not the case
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y() + 1, OPIN, opin_grid_side[1]);
        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
      

        break;
      case RIGHT: /* RIGHT = 1 */
        if (gsb_coord.x() == gsb_range.x()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for top: chany[x][y+1] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x+1][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;

        /* include Grid[x+1][y+1] Bottom side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x+1][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[1]);
        
          // Needed for 3D CB, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        
        break;
      case BOTTOM: /* BOTTOM = 2*/
        if (!perimeter_cb && gsb_coord.y() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for bottom: chany[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y] LEFT side outputs pins */
        opin_grid_side[0] = LEFT;
        /* Grid[x][y] RIGHT side outputs pins */
        opin_grid_side[1] = RIGHT;
        /* include Grid[x+1][y] Left side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Right side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        // Needed for 3D CB Outputs, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y(), OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());

        break;
      case LEFT: /* LEFT = 3 */
        if (!perimeter_cb && gsb_coord.x() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Grid[x][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;
        /* include Grid[x][y+1] Bottom side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        
        // Needed for 3D CB, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);
        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());
        
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* Organize a vector of port direction */
    if (0 < rr_chan.get_chan_width()) {
      std::vector<enum PORTS> rr_chan_dir;
      rr_chan_dir.resize(rr_chan.get_chan_width());
      for (size_t itrack = 0; itrack < rr_chan.get_chan_width(); ++itrack) {
        /* Identify the directionality, record it in rr_node_direction */
        if (Direction::INC ==
            vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[0];
        } else {
          VTR_ASSERT(Direction::DEC == vpr_device_ctx.rr_graph.node_direction(
                                         rr_chan.get_node(itrack)));
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[1];
        }
      }
      /* Fill chan_rr_nodes */
      rr_gsb.add_chan_node(side_manager.get_side(), rr_chan, rr_chan_dir);
    }

    /* Fill opin_rr_nodes */
    /* Copy from temp_opin_rr_node to opin_rr_node */
    for (size_t opin_array_id = 0; opin_array_id < temp_opin_rr_nodes.size();
         ++opin_array_id) {
      for (const RRNodeId& inode : temp_opin_rr_nodes[opin_array_id]) {
        /* Skip those has no configurable outgoing, they should NOT appear in
         * the GSB connection This is for those grid output pins used by direct
         * connections
         */
        if (0 == vpr_device_ctx.rr_graph.num_configurable_edges(inode)) {
          continue;
        }
        /* Do not consider OPINs that directly drive an IPIN
         * they are supposed to be handled by direct connection
         */
        if (true ==
            is_opin_direct_connected_ipin(vpr_device_ctx.rr_graph, inode)) {
          continue;
        }

        rr_gsb.add_opin_node(inode, side_manager.get_side());
      }
    }

    /* Clean ipin_rr_nodes */
    /* We do not have any IPIN for a Switch Block */
    rr_gsb.clear_ipin_nodes(side_manager.get_side());

    /* Clear the temp data */
    temp_opin_rr_nodes[0].clear();
    temp_opin_rr_nodes[1].clear();
    opin_grid_side[0] = NUM_2D_SIDES;
    opin_grid_side[1] = NUM_2D_SIDES;


    if (side == LEFT || side == BOTTOM){
      temp_cb_output_nodes = find_interlayer_nodes_connected_to_other_layers(
        vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, rr_chan, gsb_coord.x(), gsb_coord.y());

      // Insert found nodes into either the x or y interlayer CB outputs, ensuring no duplicates
      if (side == LEFT){
        for (auto& node: temp_cb_output_nodes){
          if (interlayer_cb_output_x.end() == std::find(interlayer_cb_output_x.begin(), interlayer_cb_output_x.end(), node)){
            interlayer_cb_output_x.push_back(node);
          }
        }
      }
      else{
        for (auto& node: temp_cb_output_nodes){
          if (interlayer_cb_output_y.end() == std::find(interlayer_cb_output_y.begin(), interlayer_cb_output_y.end(), node)){
            interlayer_cb_output_y.push_back(node);
          }
        }
      }
    }

  }

  // sort the interlayer channels to ensure they are in the same order as the input channel in the other layer
  std::sort(interlayer_cb_output_x.begin(), interlayer_cb_output_x.end());
  std::sort(interlayer_cb_output_y.begin(), interlayer_cb_output_y.end());

  // Add the interlayer channels to the GSB
  RRChan interlayer_cb_output_x_chan = build_chan_from_nodes(vpr_device_ctx, interlayer_cb_output_x, CHANX);
  RRChan interlayer_cb_output_y_chan = build_chan_from_nodes(vpr_device_ctx, interlayer_cb_output_y, CHANY);

  rr_gsb.add_cb_output_chan(CHANX, interlayer_cb_output_x_chan);
  rr_gsb.add_cb_output_chan(CHANY, interlayer_cb_output_y_chan);

  std::vector<RRNodeId> interlayer_chans_cb_input_x;
  std::vector<RRNodeId> interlayer_chans_cb_input_y;

  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);
    size_t ix;
    size_t iy;
    enum e_side chan_side;
    std::vector<RRNodeId> temp_ipin_rr_nodes;
    std::vector<RRNodeId> temp_interlayer_chans_cb_input;
    enum e_side ipin_rr_node_grid_side;

    switch (side) {
      case TOP: /* TOP = 0 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* BOTTOM side INPUT Pins of Grid[x][y+1] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y() + 1;
        ipin_rr_node_grid_side = BOTTOM;
        break;
      case RIGHT: /* RIGHT = 1 */
        /* For the bording, we should take special care */
        /* Check if TOP side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* LEFT side INPUT Pins of Grid[x+1][y] */
        ix = rr_gsb.get_sb_x() + 1;
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = LEFT;
        break;
      case BOTTOM: /* BOTTOM = 2*/
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* TOP side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = TOP;
        break;
      case LEFT: /* LEFT = 3 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = RIGHT;
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* If there is no channel at this side, we skip ipin_node annotation */
    if (0 == rr_gsb.get_chan_width(chan_side)) {
      continue;
    }
    /* Collect IPIN rr_nodes*/
    temp_ipin_rr_nodes = find_rr_graph_grid_nodes(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN,
      ipin_rr_node_grid_side, include_clock);

    // Needed for 3D CB, adding the IPIN nodes from the other layers which will become outputs to the CBs 
    // TODO: Figure out how to make other CBs output this.
    // IDEA: layer 1 channels -> layer 0 CB -> layer 0 CLB Input
    // Currently not the case as to how it works. 

    
    // This retrieves all the channel nodes on this layer that connect to IPINs on other layers and adds said IPINs as outputs of the CB
    // Instead I want to do kinda the opposite, find all the channels on other layers that connect to this IPIN and add them as inputs to the CB
    // Need to be sure that these nodes only enter here to be added as outputs of the CB to IPINs not to other channels, that's the SB's job 


    temp_interlayer_chans_cb_input = find_interlayer_chans_connected_to_input(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN, ipin_rr_node_grid_side);

    // Insert found nodes into either the x or y interlayer channels, ensuring no duplicates
    if (ipin_rr_node_grid_side == TOP || ipin_rr_node_grid_side == BOTTOM){
      for (auto& node: temp_interlayer_chans_cb_input){
        if (interlayer_chans_cb_input_x.end() == std::find(interlayer_chans_cb_input_x.begin(), interlayer_chans_cb_input_x.end(), node)){
          interlayer_chans_cb_input_x.push_back(node);
        }
      }
    }
    else{
      for (auto& node: temp_interlayer_chans_cb_input){
        if (interlayer_chans_cb_input_y.end() == std::find(interlayer_chans_cb_input_y.begin(), interlayer_chans_cb_input_y.end(), node)){
          interlayer_chans_cb_input_y.push_back(node);
        }
      }
    }

    // sort the interlayer channels to ensure they are in the same order as the output channel from the other layer
    std::sort(interlayer_chans_cb_input_x.begin(), interlayer_chans_cb_input_x.end());
    std::sort(interlayer_chans_cb_input_y.begin(), interlayer_chans_cb_input_y.end());    

    // Instead of adding them as an output of the CB, need to send the channels that connect to that CB to the corresponding layer's CB as an input
    // Maybe the way to do this is to look at the interlayer channels that are connected to the IPINs and add them to the CBs that are on the same layer?
    // Admittedly this is a bit dumnb (too many vertical connections) but this is model A aka the naive model
    // The grand idea is that the CB corresponding to this input, muxes everything into 1 signal that is sent down to this CB and then sent as input (less vertical connections)
    // implementation of this idea is not so trivial though hence why for now we are doing this

    /* Fill the ipin nodes of RRGSB */
    for (const RRNodeId& inode : temp_ipin_rr_nodes) {
      /* Skip those has no configurable outgoing, they should NOT appear in the
       * GSB connection This is for those grid output pins used by direct
       * connections
       */
      if (0 ==
          vpr_device_ctx.rr_graph.node_configurable_in_edges(inode).size()) {
        continue;
      }

      /* Do not consider IPINs that are directly connected by an OPIN
       * they are supposed to be handled by direct connection
       */
      if (true ==
          is_ipin_direct_connected_opin(vpr_device_ctx.rr_graph, inode)) {
        continue;
      }

      rr_gsb.add_ipin_node(inode, side_manager.get_side());
    }
    
    /* Clear the temp data */
    temp_ipin_rr_nodes.clear();
  }

  vtr::Point<size_t> cb_coord(rr_gsb.get_sb_x(), rr_gsb.get_sb_y());

  RRChan interlayer_cb_input_x = build_chan_from_nodes(vpr_device_ctx, interlayer_chans_cb_input_x, CHANX);

  // Left, Right for CBX
  rr_gsb.add_cb_input_chan(CHANX, interlayer_cb_input_x);


  RRChan interlayer_cb_input_y = build_chan_from_nodes(vpr_device_ctx, interlayer_chans_cb_input_y, CHANY);

  // Top, Bottom for CBY
  rr_gsb.add_cb_input_chan(CHANY, interlayer_cb_input_y);

  /* Build OPIN node lists for connection blocks */
  rr_gsb.build_cb_opin_nodes(vpr_device_ctx.rr_graph);

  return rr_gsb;
}

/* Build a 3D General Switch Block (GSB)
 * which includes:
 * [I] A Switch Box subckt consists of following ports:
 * 1. Channel Y [layer][x][y] inputs
 * 2. Channel X [layer][x+1][y] inputs
 * 3. Channel Y [layer][x][y-1] outputs
 * 4. Channel X [layer][x][y] outputs
 * 5. Grid[layer][x][y+1] Right side outputs pins
 * 6. Grid[layer][x+1][y+1] Left side output pins
 * 7. Grid[layer][x+1][y+1] Bottom side output pins
 * 8. Grid[layer][x+1][y] Top side output pins
 * 9. Grid[layer][x+1][y] Left side output pins
 * 10. Grid[layer][x][y] Right side output pins
 * 11. Grid[layer][x][y] Top side output pins
 * 12. Grid[layer][x][y+1] Bottom side output pins
 * 13*. Above Channel [layer+1][x][y] inputs 
 * 14*. Below Channel [layer-1][x][y] inputs 
 * 15*. Above Channel [layer+1][x][y] outputs 
 * 16*. Below Channel [layer-1][x][y] outputs 
 * 
 * *: Only exists if there is a layer above/below
 *
 *    --------------          --------------
 *    |            |   CBY    |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |  [x][y+1]  | [x][y+1] | [x+1][y+1] |
 *    |            |          |            |
 *    --------------          --------------
 *                  ----------
 *     ChanX & CBX  | Switch |     ChanX
 *       [x][y]     |   Box  |    [x+1][y]
 *                  | [x][y] |
 *                  ----------
 *    --------------          --------------
 *    |            |          |            |
 *    |    Grid    |  ChanY   |    Grid    |
 *    |   [x][y]   |  [x][y]  |  [x+1][y]  |
 *    |            |          |            |
 *    --------------          --------------
 * For channels chanY with INC_DIRECTION on the top side, they should be marked as outputs 
 * For channels chanY with DEC_DIRECTION on the top side, they should be marked as inputs 
 * For channels chanY with INC_DIRECTION on the bottom side, they should be marked as inputs 
 * For channels chanY with DEC_DIRECTION on the bottom side, they should be marked as outputs 
 * For channels chanX with INC_DIRECTION on the left side, they should be marked as inputs 
 * For channels chanX with DEC_DIRECTION on the left side, they should be marked as outputs
 * For channels chanX with INC_DIRECTION on the right side, they should be marked as outputs 
 * For channels chanX with DEC_DIRECTION on the right side, they should be marked as inputs
 * 
 * Vertical (Above and Below) channel nodes in the rr graph have unique properties: 
 *  1. They have no direction
 *  2. They have exactly 1 source node and 1 sink node
 *    a. One of those is on the same layer
 *    b. The other node is on a different layer
 *  3. The source and sink nodes are both channel nodes
 * 
 * How to determine whether a vertical channel should be an input or output:
 *  if the source node is on the same layer then the channel is an output of the GSB,
 *  otherwise, the channel is an input to the GSB
 * 
 * How to determine whether a vertical channel is above or below:
 *  Look at the connected node that is on a different layer,
 *  if the layer number is higher then the channel is above
 *  else, the channel is below
 *
 * [II] A X-direction Connection Block [x][y]
 * The connection block shares the same routing channel[x][y] with the Switch Block 
 * We just need to fill the ipin nodes at TOP and BOTTOM sides 
 * as well as properly fill the ipin_grid_side information 
 * 
 * [III] A Y-direction Connection Block [x][y+1] 
 * The connection block shares the same routing channel[x][y+1] with the Switch Block 
 * We just need to fill the ipin nodes at LEFT and RIGHT sides 
 * as well as properly fill the ipin_grid_side information
 */
static RRGSB build_3d_rr_gsb(const DeviceContext& vpr_device_ctx,
                          const vtr::Point<size_t>& gsb_range,
                          const size_t& layer,
                          const vtr::Point<size_t>& gsb_coord,
                          const bool& perimeter_cb, const bool& include_clock) {
  /* Create an object to return */
  RRGSB rr_gsb;

  VTR_ASSERT(gsb_coord.x() <= gsb_range.x());
  VTR_ASSERT(gsb_coord.y() <= gsb_range.y());

  /* Coordinator initialization */
  rr_gsb.set_coordinate(gsb_coord.x(), gsb_coord.y());

  size_t num_sides = 6; // 6 sides: TOP, RIGHT, BOTTOM, LEFT, ABOVE, UNDER

  /* Set the number of sides of the GSB */
  rr_gsb.init_num_sides(num_sides);

  std::vector<RRNodeId> interlayer_cb_output_x;
  std::vector<RRNodeId> interlayer_cb_output_y;
  
  /* Find all rr_nodes of horizontal channels */
  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3*/
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    /* Local variables inside this for loop */
    SideManager side_manager(side);



    vtr::Point<size_t> coordinate = gsb_coord;
    RRChan rr_chan;
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes(2);
    std::vector<std::vector<RRNodeId>> temp_opin_rr_nodes_3d(2);
    std::vector<RRNodeId> temp_cb_output_nodes;
      enum e_side opin_grid_side[2] = {NUM_2D_SIDES, NUM_2D_SIDES};

    // set SB coordinate for horizontal (non-interlayer) channels
    if (side < 4){
      coordinate = rr_gsb.get_side_block_coordinate(side_manager.get_side());
    }
    enum PORTS chan_dir_to_port_dir_mapping[2] = {
      OUT_PORT, IN_PORT}; /* 0: INC_DIRECTION => ?; 1: DEC_DIRECTION => ? */

    switch (side_manager.get_side()) {
      case e_side::TOP: /* TOP = 0 */
        if (gsb_coord.y() == gsb_range.y()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x][y+1] RIGHT side outputs pins */
        opin_grid_side[0] = RIGHT;
        /* Grid[x+1][y+1] left side outputs pins */
        opin_grid_side[1] = LEFT;
        /* Include Grid[x][y+1] RIGHT side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* Include Grid[x+1][y+1] Left side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[1]);

        // Check if there are any interlayer connections needed for 3D CB, need to check if these nodes are connected though since if the CB is 2D that is not the case
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y() + 1, OPIN, opin_grid_side[1]);

        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());

        break;

      case e_side::RIGHT: /* RIGHT = 1 */
        if (gsb_coord.x() == gsb_range.x()) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for top: chany[x][y+1] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          OUT_PORT; /* INC_DIRECTION => OUT_PORT */
        chan_dir_to_port_dir_mapping[1] =
          IN_PORT; /* DEC_DIRECTION => IN_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x+1][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;

        /* include Grid[x+1][y+1] Bottom side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x+1][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[1]);

        // Needed for 3D CB, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());

        break;

      case e_side::BOTTOM: /* BOTTOM = 2*/
        if (!perimeter_cb && gsb_coord.y() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for bottom: chany[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANY, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Assign grid side of OPIN */
        /* Grid[x+1][y] LEFT side outputs pins */
        opin_grid_side[0] = LEFT;
        /* Grid[x][y] RIGHT side outputs pins */
        opin_grid_side[1] = RIGHT;
        /* include Grid[x+1][y] Left side output pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer,
          gsb_coord.x() + 1, gsb_coord.y(), OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Right side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

          // Needed for 3D CB Outputs, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x() + 1,
          gsb_coord.y(), OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());

        break;

      case e_side::LEFT: /* LEFT = 3 */
        if (!perimeter_cb && gsb_coord.x() == 0) {
          rr_gsb.clear_one_side(side_manager.get_side());
          break;
        }
        /* Routing channels*/
        /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
        /* Collect rr_nodes for Tracks for left: chanx[x][y] */
        /* Create a rr_chan object and check if it is unique in the graph */
        rr_chan = build_one_rr_chan(vpr_device_ctx, CHANX, layer, coordinate, gsb_coord);
        chan_dir_to_port_dir_mapping[0] =
          IN_PORT; /* INC_DIRECTION => IN_PORT */
        chan_dir_to_port_dir_mapping[1] =
          OUT_PORT; /* DEC_DIRECTION => OUT_PORT */

        /* Build the Switch block: opin and opin_grid_side */
        /* Grid[x][y+1] BOTTOM side outputs pins */
        opin_grid_side[0] = BOTTOM;
        /* Grid[x][y] TOP side outputs pins */
        opin_grid_side[1] = TOP;
        /* include Grid[x][y+1] Bottom side outputs pins */
        temp_opin_rr_nodes[0] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        /* include Grid[x][y] Top side output pins */
        temp_opin_rr_nodes[1] = find_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

          // Needed for 3D CB, adding the OPIN nodes from the other layers which will become inputs to the SBs
        temp_opin_rr_nodes_3d[0] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y() + 1, OPIN, opin_grid_side[0]);
        temp_opin_rr_nodes_3d[1] = find_interlayer_rr_graph_grid_nodes(
          vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, gsb_coord.x(),
          gsb_coord.y(), OPIN, opin_grid_side[1]);

        temp_opin_rr_nodes[0].insert(temp_opin_rr_nodes[0].end(), temp_opin_rr_nodes_3d[0].begin(), temp_opin_rr_nodes_3d[0].end());
        temp_opin_rr_nodes[1].insert(temp_opin_rr_nodes[1].end(), temp_opin_rr_nodes_3d[1].begin(), temp_opin_rr_nodes_3d[1].end());

        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* Organize a vector of port direction */
    if (0 < rr_chan.get_chan_width()) {
      std::vector<enum PORTS> rr_chan_dir;
      rr_chan_dir.resize(rr_chan.get_chan_width());

      for (size_t itrack = 0; itrack < rr_chan.get_chan_width(); ++itrack) {
        /* Identify the directionality, record it in rr_node_direction */
        if (Direction::INC ==
            vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[0];
        } else if (Direction::DEC == vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = chan_dir_to_port_dir_mapping[1];
        } else if (Direction::ABOVE_INC == vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack)) || 
                    Direction::UNDER_DEC == vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = OUT_PORT;
        } else if (Direction::ABOVE_DEC == vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack)) || 
                    Direction::UNDER_INC == vpr_device_ctx.rr_graph.node_direction(rr_chan.get_node(itrack))) {
          rr_chan_dir[itrack] = IN_PORT;
        } else {
          VTR_LOG_ERROR("Invalid directionality for a channel rr_node!\n");
          exit(1);
        }
      }
      /* Fill chan_rr_nodes */
      rr_gsb.add_chan_node(side_manager.get_side(), rr_chan, rr_chan_dir);
    }

    /* Fill opin_rr_nodes */
    /* Copy from temp_opin_rr_node to opin_rr_node */
    for (size_t opin_array_id = 0; opin_array_id < temp_opin_rr_nodes.size();
         ++opin_array_id) {
      for (const RRNodeId& inode : temp_opin_rr_nodes[opin_array_id]) {
        /* Skip those has no configurable outgoing, they should NOT appear in
         * the GSB connection This is for those grid output pins used by direct
         * connections
         */
        if (0 == vpr_device_ctx.rr_graph.num_configurable_edges(inode)) {
          continue;
        }
        /* Do not consider OPINs that directly drive an IPIN
         * they are supposed to be handled by direct connection
         */
        if (true ==
            is_opin_direct_connected_ipin(vpr_device_ctx.rr_graph, inode)) {
          continue;
        }

        rr_gsb.add_opin_node(inode, side_manager.get_side());
      }
    }

    /* Clean ipin_rr_nodes */

    /* We do not have any IPIN for a Switch Block */
    rr_gsb.clear_ipin_nodes(side_manager.get_side());
    
    /* Clear the temp data */
    temp_opin_rr_nodes[0].clear();
    temp_opin_rr_nodes[1].clear();
    opin_grid_side[0] = NUM_2D_SIDES;
    opin_grid_side[1] = NUM_2D_SIDES;

    if (side == LEFT || side == BOTTOM){
      temp_cb_output_nodes = find_interlayer_nodes_connected_to_other_layers(
        vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, rr_chan, gsb_coord.x(), gsb_coord.y());

      // Insert found nodes into either the x or y interlayer CB outputs, ensuring no duplicates
      if (side == LEFT){
        for (auto& node: temp_cb_output_nodes){
          if (interlayer_cb_output_x.end() == std::find(interlayer_cb_output_x.begin(), interlayer_cb_output_x.end(), node)){
            interlayer_cb_output_x.push_back(node);
          }
        }
      }
      else{
        for (auto& node: temp_cb_output_nodes){
          if (interlayer_cb_output_y.end() == std::find(interlayer_cb_output_y.begin(), interlayer_cb_output_y.end(), node)){
            interlayer_cb_output_y.push_back(node);
          }
        }
      }
    }

  }

  // sort the interlayer channels to ensure they are in the same order as the input channel in the other layer
  std::sort(interlayer_cb_output_x.begin(), interlayer_cb_output_x.end());
  std::sort(interlayer_cb_output_y.begin(), interlayer_cb_output_y.end());

  // Add the interlayer channels to the GSB
  RRChan interlayer_cb_output_x_chan = build_chan_from_nodes(vpr_device_ctx, interlayer_cb_output_x, CHANX);
  RRChan interlayer_cb_output_y_chan = build_chan_from_nodes(vpr_device_ctx, interlayer_cb_output_y, CHANY);

  rr_gsb.add_cb_output_chan(CHANX, interlayer_cb_output_x_chan);
  rr_gsb.add_cb_output_chan(CHANY, interlayer_cb_output_y_chan);

  // Add vertical channels to the GSB
  RRChan above_rr_chan = build_one_interlayer_rr_chan(vpr_device_ctx, layer, gsb_coord, e_side::ABOVE, gsb_coord);

  if (0 < above_rr_chan.get_chan_width()) {
    std::vector<enum PORTS> rr_chan_dir;
    rr_chan_dir.resize(above_rr_chan.get_chan_width());
    for (size_t itrack = 0; itrack < above_rr_chan.get_chan_width(); ++itrack) {
      /* Identify the directionality, record it in rr_node_direction */
      if (Direction::ABOVE_DEC ==
          vpr_device_ctx.rr_graph.node_direction(above_rr_chan.get_node(itrack))) {
        rr_chan_dir[itrack] = IN_PORT;
      } else {
        VTR_ASSERT(Direction::ABOVE_INC == vpr_device_ctx.rr_graph.node_direction(
                                       above_rr_chan.get_node(itrack)));
        rr_chan_dir[itrack] = OUT_PORT;
      }
    }
    /* Fill chan_rr_nodes */
    rr_gsb.add_chan_node(e_side::ABOVE, above_rr_chan, rr_chan_dir);
  }

  RRChan under_rr_chan = build_one_interlayer_rr_chan(vpr_device_ctx, layer, gsb_coord, e_side::UNDER, gsb_coord);

  if (0 < under_rr_chan.get_chan_width()) {
    std::vector<enum PORTS> rr_chan_dir;
    rr_chan_dir.resize(under_rr_chan.get_chan_width());
    for (size_t itrack = 0; itrack < under_rr_chan.get_chan_width(); ++itrack) {
      /* Identify the directionality, record it in rr_node_direction */
      if (Direction::UNDER_INC ==
          vpr_device_ctx.rr_graph.node_direction(under_rr_chan.get_node(itrack))) {
        rr_chan_dir[itrack] = IN_PORT;
      } else {
        VTR_ASSERT(Direction::UNDER_DEC == vpr_device_ctx.rr_graph.node_direction(
                                       under_rr_chan.get_node(itrack)));
        rr_chan_dir[itrack] = OUT_PORT;
      }
    }
    /* Fill chan_rr_nodes */
    rr_gsb.add_chan_node(e_side::UNDER, under_rr_chan, rr_chan_dir);
  }

  std::vector<RRNodeId> interlayer_chans_cb_input_x;
  std::vector<RRNodeId> interlayer_chans_cb_input_y;

  /* Side: TOP => 0, RIGHT => 1, BOTTOM => 2, LEFT => 3 */
  for (size_t side = 0; side < rr_gsb.get_num_sides(); ++side) {
    // only consider the 4 sides of the GSB that are not interlayer
    // if (side > 3){
    //   break;
    // }

    /* Local variables inside this for loop */
    SideManager side_manager(side);
    if (side > 3) break;

    size_t ix;
    size_t iy;
    enum e_side chan_side;
    std::vector<RRNodeId> temp_ipin_rr_nodes;
    std::vector<RRNodeId> temp_interlayer_chans_cb_input;
    enum e_side ipin_rr_node_grid_side;

    switch (side_manager.get_side()) {
      case TOP: /* TOP = 0 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* BOTTOM side INPUT Pins of Grid[x][y+1] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y() + 1;
        ipin_rr_node_grid_side = BOTTOM;
        break;
      case RIGHT: /* RIGHT = 1 */
        /* For the bording, we should take special care */
        /* Check if TOP side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* LEFT side INPUT Pins of Grid[x+1][y] */
        ix = rr_gsb.get_sb_x() + 1;
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = LEFT;
        break;
      case BOTTOM: /* BOTTOM = 2*/
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = LEFT;
        /* Build the connection block: ipin and ipin_grid_side */
        /* TOP side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = TOP;
        break;
      case LEFT: /* LEFT = 3 */
        /* For the bording, we should take special care */
        /* Check if left side chan width is 0 or not */
        chan_side = BOTTOM;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = RIGHT;
        break;
      case ABOVE: /* ABOVE = 4 */
        chan_side = ABOVE;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = UNDER;
        break;
      case UNDER: /* UNDER = 5 */
        chan_side = UNDER;
        /* Build the connection block: ipin and ipin_grid_side */
        /* RIGHT side INPUT Pins of Grid[x][y] */
        ix = rr_gsb.get_sb_x();
        iy = rr_gsb.get_sb_y();
        ipin_rr_node_grid_side = ABOVE;
        break;
      default:
        VTR_LOG_ERROR("Invalid side index!\n");
        exit(1);
    }

    /* If there is no channel at this side, we skip ipin_node annotation */
    if (0 == rr_gsb.get_chan_width(chan_side)) {
      continue;
    }
    
    /* Collect IPIN rr_nodes*/
    temp_ipin_rr_nodes = find_rr_graph_grid_nodes(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN,
      ipin_rr_node_grid_side, include_clock);

    temp_interlayer_chans_cb_input = find_interlayer_chans_connected_to_input(
      vpr_device_ctx.rr_graph, vpr_device_ctx.grid, layer, ix, iy, IPIN, ipin_rr_node_grid_side);

    // Insert found nodes into either the x or y interlayer channels, ensuring no duplicates
    if (ipin_rr_node_grid_side == TOP || ipin_rr_node_grid_side == BOTTOM){
      for (auto& node: temp_interlayer_chans_cb_input){
        if (interlayer_chans_cb_input_x.end() == std::find(interlayer_chans_cb_input_x.begin(), interlayer_chans_cb_input_x.end(), node)){
          interlayer_chans_cb_input_x.push_back(node);
        }
      }
    }
    else{
      for (auto& node: temp_interlayer_chans_cb_input){
        if (interlayer_chans_cb_input_y.end() == std::find(interlayer_chans_cb_input_y.begin(), interlayer_chans_cb_input_y.end(), node)){
          interlayer_chans_cb_input_y.push_back(node);
        }
      }
    }

    // sort the interlayer channels to ensure they are in the same order as the output channel from the other layer
    std::sort(interlayer_chans_cb_input_x.begin(), interlayer_chans_cb_input_x.end());
    std::sort(interlayer_chans_cb_input_y.begin(), interlayer_chans_cb_input_y.end());    

    

    /* Fill the ipin nodes of RRGSB */
    for (const RRNodeId& inode : temp_ipin_rr_nodes) {
      /* Skip those has no configurable outgoing, they should NOT appear in the
       * GSB connection This is for those grid output pins used by direct
       * connections
       */
      if (0 ==
          vpr_device_ctx.rr_graph.node_configurable_in_edges(inode).size()) {
        continue;
      }

      /* Do not consider IPINs that are directly connected by an OPIN
       * they are supposed to be handled by direct connection
       */
      if (true ==
          is_ipin_direct_connected_opin(vpr_device_ctx.rr_graph, inode)) {
        continue;
      }

      rr_gsb.add_ipin_node(inode, side_manager.get_side());
    }
    
    /* Clear the temp data */
    temp_ipin_rr_nodes.clear();
  }

  vtr::Point<size_t> cb_coord(rr_gsb.get_sb_x(), rr_gsb.get_sb_y());

  RRChan interlayer_cb_input_x = build_chan_from_nodes(vpr_device_ctx, interlayer_chans_cb_input_x, CHANX);

  // Left, Right for CBX
  rr_gsb.add_cb_input_chan(CHANX, interlayer_cb_input_x);


  RRChan interlayer_cb_input_y = build_chan_from_nodes(vpr_device_ctx, interlayer_chans_cb_input_y, CHANY);

  // Top, Bottom for CBY
  rr_gsb.add_cb_input_chan(CHANY, interlayer_cb_input_y);

  /* Build OPIN node lists for connection blocks */
  rr_gsb.build_cb_opin_nodes(vpr_device_ctx.rr_graph);

  return rr_gsb;
}

/********************************************************************
 * Build the annotation for the routing resource graph
 * by collecting the nodes to the General Switch Block context
 *******************************************************************/
void annotate_device_rr_gsb(const DeviceContext& vpr_device_ctx,
                            DeviceRRGSB& device_rr_gsb,
                            const bool& include_clock,
                            const bool& verbose_output,
                            const bool is_3d_sb) {
  vtr::ScopedStartFinishTimer timer(
    "Build General Switch Block(GSB) annotation on top of routing resource "
    "graph");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range(vpr_device_ctx.grid.width() - 1,
                               vpr_device_ctx.grid.height() - 1);
  size_t layers = vpr_device_ctx.grid.get_num_layers();
  if (vpr_device_ctx.arch->perimeter_cb) {
    gsb_range.set(vpr_device_ctx.grid.width(), vpr_device_ctx.grid.height());
  }
  device_rr_gsb.reserve(gsb_range, layers);

  VTR_LOGV(verbose_output, "Start annotation GSB up to [%lu][%lu][%lu]\n",
           layers - 1, gsb_range.x() - 1, gsb_range.y() - 1);

  size_t gsb_cnt = 0;
  /* For each switch block, determine the size of array */
  for (size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        /* Here we give the builder the fringe coordinates so that it can handle
        * the GSBs at the borderside correctly sort drive_rr_nodes should be
        * called if required by users
        */
        vtr::Point<size_t> sub_gsb_range(vpr_device_ctx.grid.width() - 1,
                                        vpr_device_ctx.grid.height() - 1);

        if (is_3d_sb)
        {
          const RRGSB& rr_gsb = build_3d_rr_gsb(
            vpr_device_ctx, sub_gsb_range, ilayer, vtr::Point<size_t>(ix, iy),
            vpr_device_ctx.arch->perimeter_cb, include_clock);

          /* Add to device_rr_gsb */
          vtr::Point<size_t> gsb_coordinate = rr_gsb.get_sb_coordinate();

          device_rr_gsb.add_rr_gsb(gsb_coordinate, rr_gsb, ilayer);

          gsb_cnt++; /* Update counter */
          /* Print info */
          VTR_LOG("[%lu%] Backannotated GSB[%lu][%lu][%lu]\r",
                  100 * gsb_cnt / (gsb_range.x() * gsb_range.y() * layers), ilayer, ix, iy);
        } 

        else
        {
          const RRGSB& rr_gsb = build_rr_gsb(
            vpr_device_ctx, sub_gsb_range, ilayer, vtr::Point<size_t>(ix, iy),
            vpr_device_ctx.arch->perimeter_cb, include_clock);

          /* Add to device_rr_gsb */
          vtr::Point<size_t> gsb_coordinate = rr_gsb.get_sb_coordinate();

          device_rr_gsb.add_rr_gsb(gsb_coordinate, rr_gsb, ilayer);

          gsb_cnt++; /* Update counter */
          /* Print info */
          VTR_LOG("[%lu%] Backannotated GSB[%lu][%lu][%lu]\r",
                  100 * gsb_cnt / (gsb_range.x() * gsb_range.y() * layers), ilayer, ix, iy);
        }

      }
    }
  }
  /* Report number of unique mirrors */
  VTR_LOG("Backannotated %d General Switch Blocks (GSBs).\n",
          gsb_range.x() * gsb_range.y() * layers);
}

/********************************************************************
 * Sort all the incoming edges for each channel node which are
 * output ports of the GSB
 *******************************************************************/
void sort_device_rr_gsb_chan_node_in_edges(const RRGraphView& rr_graph,
                                           DeviceRRGSB& device_rr_gsb,
                                           const bool& verbose_output) {
  vtr::ScopedStartFinishTimer timer(
    "Sort incoming edges for each routing track output node of General Switch "
    "Block(GSB)");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range = device_rr_gsb.get_gsb_range();
  size_t layers = device_rr_gsb.get_gsb_layers();

  VTR_LOGV(verbose_output, "Start sorting edges for GSBs up to [%lu][%lu][%lu]\n",
          layers, gsb_range.x(), gsb_range.y());

  size_t gsb_cnt = 0;

  /* For each switch block, determine the size of array */
  for (size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        vtr::Point<size_t> gsb_coordinate(ix, iy);
        RRGSB& rr_gsb = device_rr_gsb.get_mutable_gsb(gsb_coordinate, ilayer);
        rr_gsb.sort_chan_node_in_edges(rr_graph);

        gsb_cnt++; /* Update counter */

        /* Print info */
        VTR_LOG(
          "[%lu%] Sorted incoming edges for each routing track output node of "
          "GSB[%lu][%lu][%lu]\r",
          100 * gsb_cnt / (layers * gsb_range.x() * gsb_range.y()), ilayer, ix, iy);
      }
    }
  }

  /* Report number of unique mirrors */
  VTR_LOG(
    "Sorted incoming edges for each routing track output node of %d General "
    "Switch Blocks (GSBs).\n",
    layers * gsb_range.x() * gsb_range.y());
}

/********************************************************************
 * Sort all the incoming edges for each input pin node which are
 * output ports of the GSB
 *******************************************************************/
void sort_device_rr_gsb_ipin_node_in_edges(const RRGraphView& rr_graph,
                                           DeviceRRGSB& device_rr_gsb,
                                           const bool& verbose_output) {
  vtr::ScopedStartFinishTimer timer(
    "Sort incoming edges for each input pin node of General Switch Block(GSB)");

  /* Note that the GSB array is smaller than the grids by 1 column and 1 row!!!
   */
  vtr::Point<size_t> gsb_range = device_rr_gsb.get_gsb_range();
  size_t layers = device_rr_gsb.get_gsb_layers();

  VTR_LOGV(verbose_output, "Start sorting edges for GSBs up to [%lu][%lu][%lu]\n",
           layers, gsb_range.x(), gsb_range.y());

  size_t gsb_cnt = 0;

  /* For each switch block, determine the size of array */
  for(size_t ilayer = 0; ilayer < layers; ++ilayer){
    for (size_t ix = 0; ix < gsb_range.x(); ++ix) {
      for (size_t iy = 0; iy < gsb_range.y(); ++iy) {
        vtr::Point<size_t> gsb_coordinate(ix, iy);
        RRGSB& rr_gsb = device_rr_gsb.get_mutable_gsb(gsb_coordinate, ilayer);
        rr_gsb.sort_ipin_node_in_edges(rr_graph);

        gsb_cnt++; /* Update counter */

        /* Print info */
        VTR_LOG(
          "[%lu%] Sorted incoming edges for each input pin node of "
          "GSB[%lu][%lu][%lu]\r",
          100 * gsb_cnt / (layers * gsb_range.x() * gsb_range.y()), ilayer, ix, iy);
      }
    }
  }

  /* Report number of unique mirrors */
  VTR_LOG(
    "Sorted incoming edges for each input pin node of %d General Switch Blocks "
    "(GSBs).\n",
    layers * gsb_range.x() * gsb_range.y());
}

/********************************************************************
 * Build the link between rr_graph switches to their physical circuit models
 * The binding is done based on the name of rr_switches defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_rr_switch_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (size_t rr_switch_id = 0;
       rr_switch_id < vpr_device_ctx.rr_graph.rr_switch().size();
       rr_switch_id++) {
    std::string switch_name(
      vpr_device_ctx.rr_graph.rr_switch()[RRSwitchId(rr_switch_id)].name);
    /* Skip the delayless switch, which is only used by the edges between
     * - SOURCE and OPIN
     * - IPIN and SINK
     */
    if (switch_name == std::string(VPR_DELAYLESS_SWITCH_NAME)) {
      continue;
    }

    CircuitModelId circuit_model = CircuitModelId::INVALID();
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    if (0 < openfpga_arch.cb_switch2circuit.count(switch_name)) {
      circuit_model = openfpga_arch.cb_switch2circuit.at(switch_name);
    }
    if (0 < openfpga_arch.sb_switch2circuit.count(switch_name)) {
      if (CircuitModelId::INVALID() != circuit_model) {
        VTR_LOG_WARN(
          "Found a connection block and a switch block switch share the same "
          "name '%s' and binded to different circuit models '%s' and "
          "'%s'!\nWill use the switch block switch binding!\n",
          switch_name.c_str(),
          openfpga_arch.circuit_lib.model_name(circuit_model).c_str(),
          openfpga_arch.circuit_lib
            .model_name(openfpga_arch.sb_switch2circuit.at(switch_name))
            .c_str());
      }
      circuit_model = openfpga_arch.sb_switch2circuit.at(switch_name);
    }

    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a routing resource graph switch "
        "'%s'!\nPlease check your OpenFPGA architecture XML!\n",
        switch_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (CIRCUIT_MODEL_MUX !=
        openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a routing resource graph switch "
        "'%s'!\nPlease check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_MUX], switch_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_rr_switch_circuit_model(RRSwitchId(rr_switch_id),
                                                      circuit_model);
    VTR_LOGV(
      verbose_output,
      "Binded a routing resource graph switch '%s' to circuit model '%s'\n",
      switch_name.c_str(),
      openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu routing resource graph switches to circuit models\n",
          count);
}

/********************************************************************
 * Build the link between rr_graph routing segments to their physical circuit
 *models The binding is done based on the name of rr_segment defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_rr_segment_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (size_t iseg = 0; iseg < vpr_device_ctx.arch->Segments.size(); ++iseg) {
    std::string segment_name = vpr_device_ctx.arch->Segments[iseg].name;
    CircuitModelId circuit_model = CircuitModelId::INVALID();
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    if (0 < openfpga_arch.routing_seg2circuit.count(segment_name)) {
      circuit_model = openfpga_arch.routing_seg2circuit.at(segment_name);
    }
    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a routing segment '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        segment_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (CIRCUIT_MODEL_CHAN_WIRE !=
        openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a routing segment '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_CHAN_WIRE],
        segment_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_rr_segment_circuit_model(RRSegmentId(iseg),
                                                       circuit_model);
    VTR_LOGV(verbose_output,
             "Binded a routing segment '%s' to circuit model '%s'\n",
             segment_name.c_str(),
             openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu routing segments to circuit models\n", count);
}

/********************************************************************
 * Build the link between rr_graph direct connection to their physical circuit
 *models The binding is done based on the name of directs defined in the
 * OpenFPGA arch XML
 *******************************************************************/
static void annotate_direct_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  size_t count = 0;

  for (size_t idirect = 0; idirect < vpr_device_ctx.arch->directs.size(); ++idirect) {
    std::string direct_name = vpr_device_ctx.arch->directs[idirect].name;
    /* The name-to-circuit mapping is stored in either cb_switch-to-circuit or
     * sb_switch-to-circuit, Try to find one and update the device annotation
     */
    ArchDirectId direct_id = openfpga_arch.arch_direct.direct(direct_name);
    /* Cannot find a direct, no annotation needed for this direct */
    if (ArchDirectId::INVALID() == direct_id) {
      continue;
    }

    CircuitModelId circuit_model =
      openfpga_arch.arch_direct.circuit_model(direct_id);
    /* Cannot find a circuit model, error out! */
    if (CircuitModelId::INVALID() == circuit_model) {
      VTR_LOG_ERROR(
        "Fail to find a circuit model for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        direct_name.c_str());
      exit(1);
    }

    /* Check the circuit model type */
    if (openfpga_arch.arch_direct.type(direct_id) !=
          e_direct_type::PART_OF_CB &&
        CIRCUIT_MODEL_WIRE !=
          openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_WIRE], direct_name.c_str());
      exit(1);
    }
    if (openfpga_arch.arch_direct.type(direct_id) ==
          e_direct_type::PART_OF_CB &&
        CIRCUIT_MODEL_MUX !=
          openfpga_arch.circuit_lib.model_type(circuit_model)) {
      VTR_LOG_ERROR(
        "Require circuit model type '%s' for a direct connection '%s'!\nPlease "
        "check your OpenFPGA architecture XML!\n",
        CIRCUIT_MODEL_TYPE_STRING[CIRCUIT_MODEL_MUX], direct_name.c_str());
      exit(1);
    }

    /* Now update the device annotation */
    vpr_device_annotation.add_direct_annotation(idirect, direct_id);
    VTR_LOGV(verbose_output,
             "Binded a direct connection '%s' to circuit model '%s'\n",
             direct_name.c_str(),
             openfpga_arch.circuit_lib.model_name(circuit_model).c_str());
    count++;
  }

  VTR_LOG("Binded %lu direct connections to circuit models\n", count);
}

/********************************************************************
 * Build the link between
 * - rr_graph switches
 * - rr_graph segments
 * - directlist
 * to their physical circuit models
 *******************************************************************/
void annotate_rr_graph_circuit_models(
  const DeviceContext& vpr_device_ctx, const Arch& openfpga_arch,
  VprDeviceAnnotation& vpr_device_annotation, const bool& verbose_output) {
  /* Iterate over each rr_switch in the device context and bind with names */
  annotate_rr_switch_circuit_models(vpr_device_ctx, openfpga_arch,
                                    vpr_device_annotation, verbose_output);

  /* Iterate over each rr_segment in the device context and bind with names */
  annotate_rr_segment_circuit_models(vpr_device_ctx, openfpga_arch,
                                     vpr_device_annotation, verbose_output);

  /* Iterate over each direct connection in the device context and bind with
   * names */
  annotate_direct_circuit_models(vpr_device_ctx, openfpga_arch,
                                 vpr_device_annotation, verbose_output);
}

// Function to check if a given node is a vertical channel node or not
bool is_vertical_node(const RRGraphView& rr_graph, RRNodeId node_id){
  if ((rr_graph.node_type(node_id) == CHANX || rr_graph.node_type(node_id) == CHANY) 
                                  && rr_graph.node_direction(node_id) == Direction::NONE){
    return true;
  }
  return false;
}


/** 
 * Function to label the side and direction of vertical channel nodes
 * 
 *  The side and direction of a vertical channel will determine if 
 * its an input to the layer or an output based on the following table:
 * 
 *          +-----------------------------------------+
 *          | Side  |  Direction  |  input or output? | 
 *          |-------+-------------+-------------------|
 *          | ABOVE |     DEC     |      Input        | 
 *          |-------+-------------+-------------------|
 *          | ABOVE |     INC     |      Output       |
 *          |-------+-------------+-------------------| 
 *          | UNDER |     DEC     |      Output       | 
 *          |-------+-------------+-------------------|
 *          | UNDER |     INC     |      Input        |
 *          +-----------------------------------------+
 * 
 * These new side and direction labels will make it easier to create 3D GSBs
 * 
 * Vertical (Above and Below) channel nodes in the rr graph have unique properties: 
 *  1. They have no direction
 *  2. They have exactly 1 source node and 1 sink node
 *    a. One of those is on the same layer
 *    b. The other node is on a different layer
 *  3. The source and sink nodes are both channel nodes
 * 
 * These properties are used to identify them and label these vertical channels
 */
void annotate_interlayer_channels(RRGraphBuilder& rr_graph_builder, const RRGraphView& rr_graph){
  RRNodeId starting_node = (RRNodeId) 0;

  for (auto it = rr_graph_builder.rr_nodes().begin(); it != rr_graph_builder.rr_nodes().end(); ++it){
      RRNodeId node_id = it->id();

      bool vertical_node = is_vertical_node(rr_graph, node_id);

      if(!vertical_node) {
        continue;
      }

      /**
       * Plan of action:
       *  1. Get all the edges for the node
       *    a. there will be exactly 2, one where the node is a sink and the other it's a source
       *  2. If the source node is on a different layer then the node, then the node is an input
       *  3. Otherwise, its an output
       *  4. if the layer number of the differring layer is higher then label the nodes side as ABOVE
       *  5. if the layer number is smaller than label the node as BELOW
       *  6. use table above to determine what direction the node should have
       */

      RRNodeId src_node = get_src_node(rr_graph, node_id);
      RRNodeId sink_node = get_sink_node(rr_graph, node_id);

      VTR_ASSERT(src_node != RRNodeId::INVALID());
      VTR_ASSERT(sink_node != RRNodeId::INVALID());

      size_t node_layer = rr_graph.node_layer(node_id);
      size_t src_layer = rr_graph.node_layer(src_node);
      size_t sink_layer = rr_graph.node_layer(sink_node);

      if (node_layer != src_layer){
        // node is an input to the layer
        VTR_ASSERT(node_layer == sink_layer);

        if (src_layer > node_layer){
          // node is an input from layer above so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::ABOVE);
          // rr_graph_builder.add_node_side(node_id, e_side::TOP); // Adding Top instead of Above since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::ABOVE_DEC);
        }
        else {
          // node is an input from layer below so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::UNDER);
          // rr_graph_builder.add_node_side(node_id, e_side::BOTTOM); // Adding Bottom instead of Under since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::UNDER_INC);
        }

      } else{
        // node is an output of the layer
        VTR_ASSERT(node_layer != sink_layer);

        if (sink_layer > node_layer){
          // node is an output from layer above so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::ABOVE);
          // rr_graph_builder.add_node_side(node_id, e_side::TOP); // Adding Top instead of Above since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::ABOVE_INC);
        }
        else {
          // node is an output from layer below so set side and direction according to table above
          // rr_graph_builder.add_node_side(node_id, e_side::UNDER);
          // rr_graph_builder.add_node_side(node_id, e_side::BOTTOM); // Adding Bottom instead of Under since the node storage only uses the 2D sides not the 3D sides
          rr_graph_builder.set_node_direction(node_id, Direction::UNDER_DEC);
        }
      }
  }
}

} /* end namespace openfpga */
