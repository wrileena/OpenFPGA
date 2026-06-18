/************************************************************************
 * Member functions for class FabricTile
 ***********************************************************************/
#include "fabric_tile.h"

#include "build_top_module_utils.h"
#include "command_exit_codes.h"
#include "vtr_assert.h"
#include "vtr_log.h"

#define VTR_ENABLE_DEBUG_LOGGING 


/* namespace openfpga begins */
namespace openfpga {

PointWithLayer FabricTile::tile_coordinate(
  const FabricTileId& tile_id) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  return coords_[tile_id];
}

PointWithLayer FabricTile::unique_tile_coordinate(
  const FabricTileId& tile_id) const {
  PointWithLayer tile_coord = tile_coordinate(tile_id);
  FabricTileId unique_fabric_tile_id = unique_tile(tile_coord);
  return tile_coordinate(unique_fabric_tile_id);
}

FabricTileId FabricTile::find_unique_tile(const FabricTileId& tile_id) const {
  PointWithLayer tile_coord = tile_coordinate(tile_id);
  return unique_tile(tile_coord);
}

std::vector<PointWithLayer> FabricTile::pb_coordinates(
  const FabricTileId& tile_id) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  std::vector<PointWithLayer> pb_root_coords;
  pb_root_coords.reserve(pb_coords_[tile_id].size());
  for (auto curr_rect : pb_coords_[tile_id]) {
    PointWithLayer new_point;
    new_point.coordinates = curr_rect.coordinates.bottom_left();
    new_point.layer = curr_rect.layer;
    pb_root_coords.push_back(new_point);
  }
  return pb_root_coords;
}

std::vector<PointWithLayer> FabricTile::cb_coordinates(
  const FabricTileId& tile_id, const t_rr_type& cb_type) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  switch (cb_type) {
    case CHANX:
      return cbx_coords_[tile_id];
    case CHANY:
      return cby_coords_[tile_id];
    default:
      VTR_LOG("Invalid type of connection block!\n");
      exit(1);
  }
  return std::vector<PointWithLayer>();
}

std::vector<PointWithLayer> FabricTile::sb_coordinates(
  const FabricTileId& tile_id) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  return sb_coords_[tile_id];
}

FabricTileId FabricTile::unique_tile(const PointWithLayer& coord) const {
  /* Return invalid Id when out of range! */
  if (coord.layer < tile_coord2unique_tile_ids_.size()) {
      if (coord.coordinates.x() < tile_coord2unique_tile_ids_[coord.layer].size()) {
        if (coord.coordinates.y() < tile_coord2unique_tile_ids_[coord.layer][coord.coordinates.x()].size()) {
          return tile_coord2unique_tile_ids_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
        }
    }
  }
  return FabricTileId::INVALID();
}

FabricTileId FabricTile::find_tile(const PointWithLayer& coord) const {

  //print error message when the coordinate is out of range
  VTR_LOG("Inside find tile function\n");

  VTR_LOG("=== PRINTING tile_coord2id_lookup_ ===\n");

for (size_t layer = 0; layer < tile_coord2id_lookup_.size(); ++layer) {

    VTR_LOG("Layer %zu\n", layer);

    for (size_t x = 0; x < tile_coord2id_lookup_[layer].size(); ++x) {

        VTR_LOG("  X = %zu, Y-size = %zu\n",
                 x,
                 tile_coord2id_lookup_[layer][x].size());

        for (size_t y = 0;
             y < tile_coord2id_lookup_[layer][x].size();
             ++y) {

            FabricTileId id =
                tile_coord2id_lookup_[layer][x][y];

            VTR_LOG("    [%zu][%zu][%zu] = %lu\n",
                     layer,
                     x,
                     y,
                     size_t(id));
        }
    }
}


  if (tile_coord2id_lookup_.empty()) {
    VTR_LOG("Babua, tile is empty\n");
  }

    if (coord.layer >= tile_coord2id_lookup_.size()) {
    VTR_LOG(
      "Tile coordinate [%lu][%lu][%lu] exceeds the maximum range [%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), tile_coord2id_lookup_.size(),
      tile_coord2id_lookup_[0].size(), tile_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.x() >= tile_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG(
      "Tile coordinate [%lu][%lu][%lu] exceeds the maximum range [%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), tile_coord2id_lookup_.size(),
      tile_coord2id_lookup_[0].size(), tile_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.y() >= tile_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG(
      "Tile coordinate [%lu][%lu][%lu] exceeds the maximum range [%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), tile_coord2id_lookup_.size(),
      tile_coord2id_lookup_[0].size(), tile_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  return tile_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
}

FabricTileId FabricTile::find_tile_by_pb_coordinate(
  const PointWithLayer& coord) const {
  if (pb_coord2id_lookup_.empty()) {
    return FabricTileId::INVALID();
  }
  if (coord.layer >= pb_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Programmable block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), pb_coord2id_lookup_.size(),
      pb_coord2id_lookup_[0].size(), pb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.x() >= pb_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Programmable block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), pb_coord2id_lookup_.size(),
      pb_coord2id_lookup_[0].size(), pb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.y() >= pb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Programmable block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), pb_coord2id_lookup_.size(),
      pb_coord2id_lookup_[0].size(), pb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  return pb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
}

FabricTileId FabricTile::find_tile_by_cb_coordinate(
  const t_rr_type& cb_type, const PointWithLayer& coord) const {
  switch (cb_type) {
    case CHANX: {
      if (cbx_coord2id_lookup_.empty()) {
        return FabricTileId::INVALID();
      }
      if (coord.layer >= cbx_coord2id_lookup_.size()) {
        VTR_LOG_ERROR(
          "X-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cbx_coord2id_lookup_.size(),
          cbx_coord2id_lookup_[0].size(), cbx_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      if (coord.coordinates.x() >= cbx_coord2id_lookup_[coord.layer].size()) {
        VTR_LOG_ERROR(
          "X-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cbx_coord2id_lookup_.size(),
          cbx_coord2id_lookup_[0].size(), cbx_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      if (coord.coordinates.y() >= cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
        VTR_LOG_ERROR(
          "X-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cbx_coord2id_lookup_.size(),
          cbx_coord2id_lookup_[0].size(), cbx_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      return cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
    }
    case CHANY: {
      if (cby_coord2id_lookup_.empty()) {
        return FabricTileId::INVALID();
      }
      if (coord.layer >= cby_coord2id_lookup_.size()) {
        VTR_LOG_ERROR(
          "Y-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cby_coord2id_lookup_.size(),
          cby_coord2id_lookup_[0].size(), cby_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      if (coord.coordinates.x() >= cby_coord2id_lookup_[coord.layer].size()) {
        VTR_LOG_ERROR(
          "Y-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cby_coord2id_lookup_.size(),
          cby_coord2id_lookup_[0].size(), cby_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      if (coord.coordinates.y() >= cby_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
        VTR_LOG_ERROR(
          "Y-direction connection block coordinate [%lu][%lu][%lu] exceeds the "
          "maximum range [%lu][%lu][%lu]!\n", coord.layer,
          coord.coordinates.x(), coord.coordinates.y(), cby_coord2id_lookup_.size(),
          cby_coord2id_lookup_[0].size(), cby_coord2id_lookup_[0][0].size());
        return FabricTileId::INVALID();
      }
      return cby_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
    }
    default:
      VTR_LOG("Invalid type of connection block!\n");
      exit(1);
  }
}

FabricTileId FabricTile::find_tile_by_sb_coordinate(
  const PointWithLayer& coord) const {
  if (sb_coord2id_lookup_.empty()) {
    return FabricTileId::INVALID();
  }
  if (coord.layer >= sb_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Switch block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), sb_coord2id_lookup_.size(),
      sb_coord2id_lookup_[0].size(), sb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.x() >= sb_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Switch block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), sb_coord2id_lookup_.size(),
      sb_coord2id_lookup_[0].size(), sb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  if (coord.coordinates.y() >= sb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Switch block coordinate [%lu][%lu][%lu] exceeds the maximum range "
      "[%lu][%lu][%lu]!\n", coord.layer,
      coord.coordinates.x(), coord.coordinates.y(), sb_coord2id_lookup_.size(),
      sb_coord2id_lookup_[0].size(), sb_coord2id_lookup_[0][0].size());
    return FabricTileId::INVALID();
  }
  return sb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()];
}

bool FabricTile::pb_in_tile(const FabricTileId& tile_id,
                            const PointWithLayer& coord,
                            const bool& use_gsb_coord) const {
  if (use_gsb_coord) {
    return !pb_gsb_coords_[tile_id].empty() &&
           find_pb_index_in_tile(tile_id, coord, use_gsb_coord) !=
             pb_gsb_coords_[tile_id].size();
  }
  return !pb_coords_[tile_id].empty() &&
         find_pb_index_in_tile(tile_id, coord) != pb_coords_[tile_id].size();
}

size_t FabricTile::find_pb_index_in_tile(const FabricTileId& tile_id,
                                         const PointWithLayer& coord,
                                         const bool& use_gsb_coord) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  if (use_gsb_coord) {
    for (size_t idx = 0; idx < pb_gsb_coords_[tile_id].size(); ++idx) {
      PointWithLayer curr_coord = pb_gsb_coords_[tile_id][idx];
      if (curr_coord.layer == coord.layer && curr_coord.coordinates == coord.coordinates) {
        return idx;
      }
    }
    /* Not found, return an invalid index */
    return pb_gsb_coords_[tile_id].size();
  } else {
    for (size_t idx = 0; idx < pb_coords_[tile_id].size(); ++idx) {
      RectWithLayer curr_coord = pb_coords_[tile_id][idx];
      if (curr_coord.layer == coord.layer && curr_coord.coordinates.coincident(coord.coordinates)) {
        return idx;
      }
    }
    /* Not found, return an invalid index */
    return pb_coords_[tile_id].size();
  }
}

bool FabricTile::sb_in_tile(const FabricTileId& tile_id,
                            const PointWithLayer& coord) const {
  return !sb_coords_[tile_id].empty() &&
         find_sb_index_in_tile(tile_id, coord) != sb_coords_[tile_id].size();
}

size_t FabricTile::find_sb_index_in_tile(
  const FabricTileId& tile_id, const PointWithLayer& coord) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  for (size_t idx = 0; idx < sb_coords_[tile_id].size(); ++idx) {
    PointWithLayer curr_coord = sb_coords_[tile_id][idx];
    if (curr_coord.layer == coord.layer && curr_coord.coordinates == coord.coordinates) {
      return idx;
    }
  }
  /* Not found, return an invalid index */
  return sb_coords_[tile_id].size();
}

bool FabricTile::cb_in_tile(const FabricTileId& tile_id,
                            const t_rr_type& cb_type,
                            const PointWithLayer& coord) const {
  switch (cb_type) {
    case CHANX:
      return !cbx_coords_[tile_id].empty() &&
             find_cb_index_in_tile(tile_id, cb_type, coord) !=
               cbx_coords_[tile_id].size();
    case CHANY:
      return !cby_coords_[tile_id].empty() &&
             find_cb_index_in_tile(tile_id, cb_type, coord) !=
               cby_coords_[tile_id].size();
    default:
      VTR_LOG("Invalid type of connection block!\n");
      exit(1);
  }
}

size_t FabricTile::find_cb_index_in_tile(
  const FabricTileId& tile_id, const t_rr_type& cb_type,
  const PointWithLayer& coord) const {
  VTR_ASSERT(valid_tile_id(tile_id));
  switch (cb_type) {
    case CHANX:
      for (size_t idx = 0; idx < cbx_coords_[tile_id].size(); ++idx) {
        PointWithLayer curr_coord = cbx_coords_[tile_id][idx];
        if (curr_coord.layer == coord.layer && curr_coord.coordinates == coord.coordinates) {
          return idx;
        }
      }
      return cbx_coords_[tile_id].size();
    case CHANY:
      for (size_t idx = 0; idx < cby_coords_[tile_id].size(); ++idx) {
        PointWithLayer curr_coord = cby_coords_[tile_id][idx];
        if (curr_coord.layer == coord.layer && coord.coordinates == curr_coord.coordinates) {
          return idx;
        }
      }
      return cby_coords_[tile_id].size();
    default:
      VTR_LOG("Invalid type of connection block!\n");
      exit(1);
  }
}

PointWithLayer FabricTile::find_cb_coordinate_in_unique_tile(
  const FabricTileId& tile_id, const t_rr_type& cb_type,
  const PointWithLayer& cb_coord) const {
  size_t cb_idx_in_curr_tile =
    find_cb_index_in_tile(tile_id, cb_type, cb_coord);
  FabricTileId unique_tile = find_unique_tile(tile_id);
  return cb_coordinates(unique_tile, cb_type)[cb_idx_in_curr_tile];
}

PointWithLayer FabricTile::find_pb_coordinate_in_unique_tile(
  const FabricTileId& tile_id, const PointWithLayer& pb_coord) const {
  size_t pb_idx_in_curr_tile = find_pb_index_in_tile(tile_id, pb_coord);
  FabricTileId unique_tile = find_unique_tile(tile_id);
  return pb_coordinates(unique_tile)[pb_idx_in_curr_tile];
}

PointWithLayer FabricTile::find_sb_coordinate_in_unique_tile(
  const FabricTileId& tile_id, const PointWithLayer& sb_coord) const {
  size_t sb_idx_in_curr_tile = find_sb_index_in_tile(tile_id, sb_coord);
  FabricTileId unique_tile = find_unique_tile(tile_id);
  return sb_coordinates(unique_tile)[sb_idx_in_curr_tile];
}

std::vector<FabricTileId> FabricTile::unique_tiles() const {
  return unique_tile_ids_;
}

bool FabricTile::empty() const { return ids_.empty(); }

FabricTileId FabricTile::create_tile(const PointWithLayer& coord) {
  FabricTileId tile_id = FabricTileId(ids_.size());
  ids_.push_back(tile_id);
  coords_.push_back(coord);
  pb_coords_.emplace_back();
  pb_gsb_coords_.emplace_back();
  cbx_coords_.emplace_back();
  cby_coords_.emplace_back();
  sb_coords_.emplace_back();

  /* Register in fast look-up */
  if (register_tile_in_lookup(tile_id, coord)) {
    return tile_id;
  }
  return FabricTileId::INVALID();
}

void FabricTile::init(const PointWithLayer& max_coord) {

  //print the max_coord for debugging
  VTR_LOG("Initializing FabricTile with max_coord: layer=%lu, x=%lu, y=%lu\n",
          max_coord.layer, max_coord.coordinates.x(), max_coord.coordinates.y());

  //It's not entering the function at all 


  tile_coord2id_lookup_.resize(max_coord.layer);
  pb_coord2id_lookup_.resize(max_coord.layer);
  cbx_coord2id_lookup_.resize(max_coord.layer);
  cby_coord2id_lookup_.resize(max_coord.layer);
  sb_coord2id_lookup_.resize(max_coord.layer);
  for (size_t ilayer=0; ilayer < max_coord.layer; ++ilayer) {
    tile_coord2id_lookup_[ilayer].resize(max_coord.coordinates.x());
    pb_coord2id_lookup_[ilayer].resize(max_coord.coordinates.x());
    cbx_coord2id_lookup_[ilayer].resize(max_coord.coordinates.x());
    cby_coord2id_lookup_[ilayer].resize(max_coord.coordinates.x());
    sb_coord2id_lookup_[ilayer].resize(max_coord.coordinates.x());
    for (size_t ix = 0; ix < max_coord.coordinates.x(); ++ix) {
      tile_coord2id_lookup_[ilayer][ix].resize(max_coord.coordinates.y(), FabricTileId::INVALID());
      pb_coord2id_lookup_[ilayer][ix].resize(max_coord.coordinates.y(), FabricTileId::INVALID());
      cbx_coord2id_lookup_[ilayer][ix].resize(max_coord.coordinates.y(), FabricTileId::INVALID());
      cby_coord2id_lookup_[ilayer][ix].resize(max_coord.coordinates.y(), FabricTileId::INVALID());
      sb_coord2id_lookup_[ilayer][ix].resize(max_coord.coordinates.y(), FabricTileId::INVALID());
    }
  }
  tile_coord2unique_tile_ids_.resize(max_coord.layer);
  for (size_t ilayer = 0; ilayer < max_coord.layer; ++ilayer) {
    tile_coord2unique_tile_ids_[ilayer].resize(max_coord.coordinates.x());
    for (size_t ix = 0; ix < max_coord.coordinates.x(); ++ix) {
      tile_coord2unique_tile_ids_[ilayer][ix].resize(max_coord.coordinates.y(),
                                            FabricTileId::INVALID());
    }
  }
}

bool FabricTile::register_tile_in_lookup(const FabricTileId& tile_id,
                                         const PointWithLayer& coord) {
  if (coord.layer >= tile_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given layer='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.layer, tile_coord2id_lookup_.size());
    return false;
  }
  if (coord.coordinates.x() >= tile_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given x='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.x(), tile_coord2id_lookup_[coord.layer].size());
    return false;
  }
  if (coord.coordinates.y() >= tile_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given y='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.y(), tile_coord2id_lookup_[coord.layer][coord.coordinates.x()].size());
    return false;
  }
  /* Throw error if this coord is already registered! */
  if (tile_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()]) {
    VTR_LOG_ERROR("Tile at [%lu][%lu][%lu] has already been registered!\n",
                  coord.layer, coord.coordinates.x(), coord.coordinates.y());
    return false;
  }
  tile_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = tile_id;

  return true;
}

bool FabricTile::register_pb_in_lookup(const FabricTileId& tile_id,
                                       const PointWithLayer& coord) {
  if (coord.layer >= pb_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given layer='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.layer, pb_coord2id_lookup_.size());
    return false;
  }
  if (coord.coordinates.x() >= pb_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given x='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.x(), pb_coord2id_lookup_[coord.layer].size());
    return false;
  }
  if (coord.coordinates.y() >= pb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given y='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.y(), pb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size());
    return false;
  }
  /* Throw error if this coord is already registered! */
  if (pb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] &&
      pb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] != tile_id) {
    VTR_LOG_ERROR(
      "Programmable block at [%lu][%lu][%lu] has already been registered!\n",
      coord.layer, coord.coordinates.x(), coord.coordinates.y());
    return false;
  }
  pb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = tile_id;

  return true;
}

bool FabricTile::register_cbx_in_lookup(const FabricTileId& tile_id,
                                        const PointWithLayer& coord) {
  if (coord.layer >= cbx_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given layer='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.layer, cbx_coord2id_lookup_.size());
    return false;
  }
  if (coord.coordinates.x() >= cbx_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given x='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.x(), cbx_coord2id_lookup_[coord.layer].size());
    return false;
  }
  if (coord.coordinates.y() >= cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given y='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.y(), cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()].size());
    return false;
  }
  /* Throw error if this coord is already registered! */
  if (cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()]) {
    VTR_LOG_ERROR(
      "X-direction connection block at [%lu][%lu][%lu] has already been "
      "registered!\n",
      coord.layer, coord.coordinates.x(), coord.coordinates.y());
    return false;
  }
  cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = tile_id;

  return true;
}

bool FabricTile::register_cby_in_lookup(const FabricTileId& tile_id,
                                        const PointWithLayer& coord) {
  if (coord.layer >= cby_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given layer='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.layer, cby_coord2id_lookup_.size());
    return false;
  }
  if (coord.coordinates.x() >= cby_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given x='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.x(), cby_coord2id_lookup_[coord.layer].size());
    return false;
  }
  if (coord.coordinates.y() >= cby_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given y='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.y(), cby_coord2id_lookup_[coord.layer][coord.coordinates.x()].size());
    return false;
  }
  /* Throw error if this coord is already registered! */
  if (cby_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()]) {
    VTR_LOG_ERROR(
      "Y-direction connection block at [%lu][%lu][%lu] has already been "
      "registered!\n",
      coord.layer, coord.coordinates.x(), coord.coordinates.y());
    return false;
  }
  cby_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = tile_id;

  return true;
}

bool FabricTile::register_sb_in_lookup(const FabricTileId& tile_id,
                                       const PointWithLayer& coord) {
  if (coord.layer >= sb_coord2id_lookup_.size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given layer='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.layer, sb_coord2id_lookup_.size());
    return false;
  }
  if (coord.coordinates.x() >= sb_coord2id_lookup_[coord.layer].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given x='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.x(), sb_coord2id_lookup_[coord.layer].size());
    return false;
  }
  if (coord.coordinates.y() >= sb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size()) {
    VTR_LOG_ERROR(
      "Fast look-up has not been re-allocated properly! Given y='%lu' exceeds "
      "the upper-bound '%lu'!\n",
      coord.coordinates.y(), sb_coord2id_lookup_[coord.layer][coord.coordinates.x()].size());
    return false;
  }
  /* Throw error if this coord is already registered! */
  if (sb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()]) {
    VTR_LOG_ERROR("Switch block at [%lu][%lu][%lu] has already been registered!\n",
                  coord.layer, coord.coordinates.x(), coord.coordinates.y());
    return false;
  }
  sb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = tile_id;

  return true;
}

void FabricTile::invalidate_tile_in_lookup(const PointWithLayer& coord) {
  tile_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = FabricTileId::INVALID();
}

void FabricTile::invalidate_pb_in_lookup(const PointWithLayer& coord) {
  pb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = FabricTileId::INVALID();
}

void FabricTile::invalidate_cbx_in_lookup(const PointWithLayer& coord) {
  cbx_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = FabricTileId::INVALID();
}

void FabricTile::invalidate_cby_in_lookup(const PointWithLayer& coord) {
  cby_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = FabricTileId::INVALID();
}

void FabricTile::invalidate_sb_in_lookup(const PointWithLayer& coord) {
  sb_coord2id_lookup_[coord.layer][coord.coordinates.x()][coord.coordinates.y()] = FabricTileId::INVALID();
}

bool FabricTile::set_tile_coordinate(const FabricTileId& tile_id,
                                     const PointWithLayer& coord) {
  VTR_ASSERT(valid_tile_id(tile_id));
  /* Invalidate previous coordinate in look-up */
  invalidate_tile_in_lookup(coords_[tile_id]);
  /* update coordinate */
  coords_[tile_id] = coord;
  /* Register in fast look-up */
  return register_tile_in_lookup(tile_id, coord);
}

int FabricTile::add_pb_coordinate(const FabricTileId& tile_id,
                                  const PointWithLayer& coord,
                                  const PointWithLayer& gsb_coord) {
  VTR_ASSERT(valid_tile_id(tile_id));
  RectWithLayer new_pb_coord;
  new_pb_coord.layer = coord.layer;
  new_pb_coord.coordinates = vtr::Rect<size_t>(coord.coordinates, coord.coordinates);
  pb_coords_[tile_id].push_back(new_pb_coord);
  pb_gsb_coords_[tile_id].push_back(gsb_coord);
  /* Register in fast look-up */
  return register_pb_in_lookup(tile_id, coord);
}

int FabricTile::set_pb_max_coordinate(const FabricTileId& tile_id,
                                      const size_t& pb_index,
                                      const PointWithLayer& max_coord) {
  VTR_ASSERT(valid_tile_id(tile_id));
  if (pb_index >= pb_coords_[tile_id].size()) {
    VTR_LOG_ERROR(
      "Invalid pb_index '%lu' is out of range of programmable block list "
      "(size='%lu') of tile [%lu][%lu][%lu]!\n",
      pb_index, pb_coords_[tile_id].size(), 
      tile_coordinate(tile_id).layer,
      tile_coordinate(tile_id).coordinates.x(),
      tile_coordinate(tile_id).coordinates.y());
    return CMD_EXEC_FATAL_ERROR;
  }
  if (max_coord.layer != pb_coords_[tile_id][pb_index].layer ||
      max_coord.coordinates.x() < pb_coords_[tile_id][pb_index].coordinates.xmin() ||
      max_coord.coordinates.y() < pb_coords_[tile_id][pb_index].coordinates.ymin()) {
    VTR_LOG_ERROR(
      "Invalid max. coordinate (%lu, %lu, %lu) is out of range of programmable "
      "block list (%lu, %lu, %lu) <-> (%lu, %lu, %lu) of tile [%lu][%lu][%lu]!\n",
      max_coord.layer, max_coord.coordinates.x(), max_coord.coordinates.y(), 
      pb_coords_[tile_id][pb_index].layer,
      pb_coords_[tile_id][pb_index].coordinates.xmin(),
      pb_coords_[tile_id][pb_index].coordinates.ymin(),
      pb_coords_[tile_id][pb_index].layer,
      pb_coords_[tile_id][pb_index].coordinates.xmax(),
      pb_coords_[tile_id][pb_index].coordinates.ymax(), 
      tile_coordinate(tile_id).layer,
      tile_coordinate(tile_id).coordinates.x(),
      tile_coordinate(tile_id).coordinates.y());
    return CMD_EXEC_FATAL_ERROR;
  }
  pb_coords_[tile_id][pb_index].coordinates.set_xmax(max_coord.coordinates.x());
  pb_coords_[tile_id][pb_index].coordinates.set_ymax(max_coord.coordinates.y());
  /* Update fast lookup */
  for (size_t ix = pb_coords_[tile_id][pb_index].coordinates.xmin();
       ix <= pb_coords_[tile_id][pb_index].coordinates.xmax(); ++ix) {
    for (size_t iy = pb_coords_[tile_id][pb_index].coordinates.ymin(); 
         iy <= pb_coords_[tile_id][pb_index].coordinates.ymax(); ++iy) {
      PointWithLayer new_point;
      new_point.layer = max_coord.layer;
      new_point.coordinates = vtr::Point<size_t>(ix, iy);
      register_pb_in_lookup(tile_id, new_point);
    }
  }
  return CMD_EXEC_SUCCESS;
}

int FabricTile::add_cb_coordinate(const FabricTileId& tile_id,
                                  const t_rr_type& cb_type,
                                  const PointWithLayer& coord) {
  VTR_ASSERT(valid_tile_id(tile_id));
  switch (cb_type) {
    case CHANX:
      cbx_coords_[tile_id].push_back(coord);
      /* Register in fast look-up */
      return register_cbx_in_lookup(tile_id, coord);
    case CHANY:
      cby_coords_[tile_id].push_back(coord);
      /* Register in fast look-up */
      return register_cby_in_lookup(tile_id, coord);
    default:
      VTR_LOG("Invalid type of connection block!\n");
      exit(1);
  }
}

int FabricTile::add_sb_coordinate(const FabricTileId& tile_id,
                                  const PointWithLayer& coord) {
  VTR_ASSERT(valid_tile_id(tile_id));
  sb_coords_[tile_id].push_back(coord);
  /* Register in fast look-up */
  return register_sb_in_lookup(tile_id, coord);
}

void FabricTile::clear() {
  ids_.clear();
  coords_.clear();
  pb_coords_.clear();
  pb_gsb_coords_.clear();
  cbx_coords_.clear();
  cby_coords_.clear();
  sb_coords_.clear();
  tile_coord2id_lookup_.clear();
  pb_coord2id_lookup_.clear();
  cbx_coord2id_lookup_.clear();
  cby_coord2id_lookup_.clear();
  sb_coord2id_lookup_.clear();
  tile_coord2unique_tile_ids_.clear();
  unique_tile_ids_.clear();
}

bool FabricTile::valid_tile_id(const FabricTileId& tile_id) const {
  return (size_t(tile_id) < ids_.size()) && (tile_id == ids_[tile_id]);
}

bool FabricTile::equivalent_tile(const FabricTileId& tile_a,
                                 const FabricTileId& tile_b,
                                 const DeviceGrid& grids,
                                 const DeviceRRGSB& device_rr_gsb) const {
  /* The number of cbx, cby and sb blocks should be the same */
  if (pb_coords_[tile_a].size() != pb_coords_[tile_b].size() ||
      pb_gsb_coords_[tile_a].size() != pb_gsb_coords_[tile_b].size() ||
      cbx_coords_[tile_a].size() != cbx_coords_[tile_b].size() ||
      cby_coords_[tile_a].size() != cby_coords_[tile_b].size() ||
      sb_coords_[tile_a].size() != sb_coords_[tile_b].size()) {
    return false;
  }

  /* The pb of two tiles should be the same, otherwise not equivalent */
  for (size_t iblk = 0; iblk < pb_coords_[tile_a].size(); ++iblk) {
    vtr::Point<size_t> tile_a_pb_coord = pb_coords_[tile_a][iblk].coordinates.bottom_left();
    vtr::Point<size_t> tile_b_pb_coord = pb_coords_[tile_b][iblk].coordinates.bottom_left();
    
    PointWithLayer tile_a_point;
    tile_a_point.layer = pb_coords_[tile_a][iblk].layer;
    tile_a_point.coordinates = tile_a_pb_coord;

    PointWithLayer tile_b_point;
    tile_b_point.layer = pb_coords_[tile_b][iblk].layer;
    tile_b_point.coordinates = tile_b_pb_coord;

    if (generate_grid_block_module_name_in_top_module(std::string(), grids,
                                                      tile_a_point.coordinates, tile_a_point.layer) !=
        generate_grid_block_module_name_in_top_module(std::string(), grids,
                                                      tile_b_point.coordinates, tile_b_point.layer)) {
      return false;
    }
  }

  /* Each CBx should have the same unique modules in the device rr_gsb */
  for (size_t iblk = 0; iblk < cbx_coords_[tile_a].size(); ++iblk) {
    if (device_rr_gsb.get_cb_unique_module_index(CHANX,
                                                 cbx_coords_[tile_a][iblk].coordinates, cbx_coords_[tile_a][iblk].layer) !=
        device_rr_gsb.get_cb_unique_module_index(CHANX,
                                                 cbx_coords_[tile_b][iblk].coordinates, cbx_coords_[tile_b][iblk].layer)) {
      return false;
    }
  }
  for (size_t iblk = 0; iblk < cby_coords_[tile_a].size(); ++iblk) {
    if (device_rr_gsb.get_cb_unique_module_index(CHANY,
                                                 cby_coords_[tile_a][iblk].coordinates, cby_coords_[tile_a][iblk].layer) !=
        device_rr_gsb.get_cb_unique_module_index(CHANY,
                                                 cby_coords_[tile_b][iblk].coordinates, cby_coords_[tile_b][iblk].layer)) {
      return false;
    }
  }
  for (size_t iblk = 0; iblk < sb_coords_[tile_a].size(); ++iblk) {
    if (device_rr_gsb.get_sb_unique_module_index(sb_coords_[tile_a][iblk].coordinates, sb_coords_[tile_a][iblk].layer) !=
        device_rr_gsb.get_sb_unique_module_index(sb_coords_[tile_b][iblk].coordinates, sb_coords_[tile_b][iblk].layer)) {
      return false;
    }
  }
  return true;
}

int FabricTile::build_unique_tiles(const DeviceGrid& grids,
                                   const DeviceRRGSB& device_rr_gsb,
                                   const bool& verbose) {
  for (size_t ilayer = 0; ilayer < (size_t)grids.get_num_layers(); ++ilayer) {
    for (size_t ix = 0; ix < grids.width(); ++ix) {
      for (size_t iy = 0; iy < grids.height(); ++iy) {
        if (!valid_tile_id(tile_coord2id_lookup_[ilayer][ix][iy])) {
          continue; /* Skip invalid tile (which does not exist) */
        }
        bool is_unique_tile = true;
        for (FabricTileId unique_tile_id : unique_tile_ids_) {
          if (equivalent_tile(tile_coord2id_lookup_[ilayer][ix][iy], unique_tile_id,
                              grids, device_rr_gsb)) {
            VTR_LOGV(verbose,
                    "Tile [%lu][%lu][%lu] is a mirror to the unique tile [%lu][%lu][%lu]\n",
                    ilayer, ix, iy, 
                    tile_coordinate(unique_tile_id).layer,
                    tile_coordinate(unique_tile_id).coordinates.x(),
                    tile_coordinate(unique_tile_id).coordinates.y());
            is_unique_tile = false;
            tile_coord2unique_tile_ids_[ilayer][ix][iy] = unique_tile_id;
            break;
          }
        }
        /* Update list if this is a unique tile */
        if (is_unique_tile) {
          VTR_LOGV(verbose, "Tile [%lu][%lu][%lu] is added as a new unique tile\n", 
                   ilayer, ix, iy);
          unique_tile_ids_.push_back(tile_coord2id_lookup_[ilayer][ix][iy]);
          tile_coord2unique_tile_ids_[ilayer][ix][iy] = tile_coord2id_lookup_[ilayer][ix][iy];
        }
      }
    }
    }
  return 0;
}

} /* End namespace openfpga*/
