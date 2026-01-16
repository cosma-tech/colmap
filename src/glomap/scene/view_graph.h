#pragma once

#include "glomap/scene/image.h"
#include "glomap/scene/image_pair.h"
#include "glomap/scene/types.h"

#include <unordered_map>
#include <unordered_set>

namespace glomap {

struct ViewGraph {
  std::unordered_map<image_pair_t, ImagePair> image_pairs;

  // Create the adjacency list for the images in the view graph.
  std::unordered_map<image_t, std::unordered_set<image_t>>
  CreateImageAdjacencyList() const;

  // Create the adjacency list for the frames in the view graph.
  std::unordered_map<frame_t, std::unordered_set<frame_t>>
  CreateFrameAdjacencyList(
      const std::unordered_map<image_t, Image>& images) const;

  // Mark the images which are not connected to any other images as not
  // registered Returns the number of images in the largest connected component.
  int KeepLargestConnectedComponents(
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<image_t, Image>& images);

  // Keep all connected components above a minimum frame threshold.
  // Returns a map from frame_id to component_id for all kept components.
  std::unordered_map<frame_t, int> KeepConnectedComponents(
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<image_t, Image>& images,
      int min_num_frames);

  // Mark connected clusters of images, where the cluster_id is sorted by the
  // the number of images.
  int MarkConnectedComponents(std::unordered_map<frame_t, Frame>& frames,
                              std::unordered_map<image_t, Image>& images,
                              int min_num_img = -1);
};

}  // namespace glomap
