
#ifndef OctVoxMap_HPP_
#define OctVoxMap_HPP_

#include <set>
#include <list>
#include <queue>
#include <vector>
#include <memory>
#include <cstring>
#include <iostream>
#include <execution>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>

#include "tsl/robin_map.h"
#include "HKNN_list60_gem.h"


namespace LI2Sup{

template<int K, typename Point>
class KNNHeap {
  static_assert(K == 5, "KNNHeap: update_worst_unrolled is hand-unrolled for K == 5 only");
public:
  KNNHeap() : count(0), worst_(0), max_dist2_(0.0f) {
    memset(dist2_, 0, sizeof(dist2_));
  }

  void reset() {
    count = 0;
    worst_ = 0;
    max_dist2_ = 0.0f;
    memset(dist2_, 0, sizeof(dist2_));
  }

  uint8_t count;
  uint8_t worst_;
  float max_dist2_;
  float dist2_[K];
  std::array<Point, K> points_;

  inline void try_insert(float dist2, const Point& pt) {
    const bool not_full = (count < K);
    const bool should_insert = not_full || (dist2 < max_dist2_);
    
    if (should_insert) {
      const uint8_t insert_idx = not_full ? count : worst_;
      
      dist2_[insert_idx] = dist2;
      points_[insert_idx] = pt;
      
      if (not_full) {
        count++;
        if (dist2 > max_dist2_) {
          max_dist2_ = dist2;
          worst_ = insert_idx;
        }
      } else {
        update_worst_unrolled();
      }
    }
  }

private:
  inline void update_worst_unrolled() {
    float d0 = dist2_[0], d1 = dist2_[1], d2 = dist2_[2], d3 = dist2_[3], d4 = dist2_[4];
    
    uint8_t idx01 = d0 > d1 ? 0 : 1;
    float max01 = d0 > d1 ? d0 : d1;
    
    uint8_t idx23 = d2 > d3 ? 2 : 3;
    float max23 = d2 > d3 ? d2 : d3;
    
    uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
    float max0123 = max01 > max23 ? max01 : max23;
    
    worst_ = max0123 > d4 ? idx0123 : 4;
    max_dist2_ = max0123 > d4 ? max0123 : d4;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
};


template<typename Point>
class OctVox{
public:
  OctVox(const Point& pt, uint8_t local_idx)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  void AddPoint(const Point& pt, uint8_t local_idx) {
    uint8_t& count = counts_[local_idx];
    Point& stored_point = points_[local_idx];
    if(count == UNINIT_MASK) {
      stored_point = pt;
      count = 1;
      return;
    }

    if(count >= MAX_POINTS_PER_SUBVOXEL) return;
    if ((pt - stored_point).squaredNorm() > DISTANCE_THRESHOLD_SQ) return;

    stored_point = (stored_point * count + pt) / (count + 1);
    ++count;
  }

  bool getPoint(const uint8_t local_idx, Point& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  static constexpr uint8_t UNINIT_MASK = 0x00;
  static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL = 20;
  static constexpr double DISTANCE_THRESHOLD_SQ = 0.1 * 0.1;

  std::array<uint8_t, 8> counts_;
  std::array<Point, 8> points_;
};



template<typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  using KNNHeapType = KNNHeap<5, Point>;
  using OctVoxType = OctVox<Point>;

  struct Options {
    float resolution      = 0.5;
    // LRU eviction only starts once the voxel count reaches capacity
    // (~150 B per voxel: list node + robin_map slot). Configs setting this
    // to 1e8 (~15 GB) never evict in practice; long-duration large-area
    // mapping on RAM-limited targets needs a smaller value.
    std::size_t capacity  = 1000000;

    Options(float __resolution, std::size_t __capacity) {
      resolution = __resolution;
      capacity = __capacity;
    }
  };

  

  OctVoxMap() {
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }
  
  ~OctVoxMap() {
    grids_.clear();
    data_.clear();
  }
  
  OctVoxMap(Options options){
    SetOptions(options);
    std::cout << " ---> OctVoxMap init. Resolution: " << resolution_ 
              << " Capacity: " << capacity_ << std::endl;
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  void SetOptions(const Options& options)
  {
    resolution_ = options.resolution;
    capacity_ = options.capacity;
    inv_resolution_ = 1.0 / resolution_;
    sub_resolution_ = resolution_ / 2.0;
    sub_inv_resolution_ = 1.0 / sub_resolution_;
    // orders_min_dis2 holds the squared group shell bounds for resolution 0.5
    // (sub-cell 0.25). All shell distances are fixed multiples of the sub-cell
    // edge, so the squared bounds scale with (resolution_ / 0.5)^2.
    const float dis2_scale = resolution_ * resolution_ * 4.0f;
    for (std::size_t i = 0; i < orders_min_dis2.size(); ++i) {
      orders_min_dis2_scaled_[i] = orders_min_dis2[i] * dis2_scale;
    }
  }

  void insert(const Points& cloud_world);
  void printInfo() const;
  void clear();

  void getTopK(const Point& point, KNNHeapType& top_K) const;

  void reset_max_group(){
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  // size_t getMemoryUsageBytes() const {
  //   size_t bytes = 0;
  //   bytes += sizeof(*this);
  //   bytes += data_.size() * (sizeof(KEY) + sizeof(OctVoxType)
  //                           + sizeof(void*) * 2); // list node pointers
  //   bytes += grids_.size() * (sizeof(KEY) + sizeof(DATA_ITER)
  //                             + sizeof(size_t)); // hash & pair overhead
  //   bytes += grids_.bucket_count() * sizeof(void*); // bucket array
  //   bytes += flat_search_ptrs_.capacity() * sizeof(uint8_t*);
  //   return bytes;
  // }


private:
  float resolution_ = 0.5;
  float inv_resolution_ = 1.0;
  float sub_resolution_ = 0.25;
  float sub_inv_resolution_ = 4.0;
  // Per-resolution rescaling of orders_min_dis2; the default matches the
  // table (resolution 0.5) and SetOptions recomputes it.
  std::array<float, 6> orders_min_dis2_scaled_ = orders_min_dis2;
  std::size_t capacity_ = 1000000;

  /// HashShiftMix
  struct HASH_VEC {
    std::size_t operator()(const KEY &v) const {
      size_t h = static_cast<size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DATA_LIST = std::list<std::pair<KEY, OctVoxType>>;
  using DATA_ITER = typename DATA_LIST::iterator;

  DATA_LIST data_;
  tsl::robin_map<KEY, DATA_ITER, HASH_VEC> grids_;

  std::vector<uint8_t*> flat_search_ptrs_;
  int group_idx_max_;

};


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::insert(const Points& cloud_world){
  for(auto& pt : cloud_world){
    KEY fine_key = (pt * sub_inv_resolution_).array().floor().template cast<int>();
    KEY key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    uint8_t dx = fine_key[0] & 1;
    uint8_t dy = fine_key[1] & 1;
    uint8_t dz = fine_key[2] & 1;
    uint8_t local_idx = (dz << 2) | (dy << 1) | dx;

    auto iter = grids_.find(key);
    if (iter == grids_.end()) {
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx));
      grids_.insert(std::make_pair(key, data_.begin()));
      
      if (data_.size() >= capacity_) {
        grids_.erase(data_.back().first);
        data_.pop_back();
      }
    } else {
      iter->second->second.AddPoint(pt, local_idx);
      data_.splice(data_.begin(), data_, iter->second);
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK(const Point& point, KNNHeapType& top_K) const {
  const KEY fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
  KEY key;
  key[0] = fine_key[0] >> 1;
  key[1] = fine_key[1] >> 1;
  key[2] = fine_key[2] >> 1;

  const int dx = fine_key[0] & 1;
  const int dy = fine_key[1] & 1;
  const int dz = fine_key[2] & 1;
  const int local_idx = (dz << 2) | (dy << 1) | dx;
  const KEY mirror_axis = KEY(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));
  
  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);
  
  for(uint8_t i = 0; i < pre_voxel_ptr_size; ++i)
  {
    KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[i]);
    KEY n_key = key + delta_key;
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      top_voxels_2_search[i] = &iter->second->second;
    }
  }

  Point __sub_point;

  for (int group_idx = 0; group_idx < group_idx_max_; ++group_idx) {
    const uint8_t* group_it = flat_search_ptrs_[group_idx];
    const uint8_t* group_end = flat_search_ptrs_[group_idx + 1];

    while(group_it < group_end){
      const uint8_t neighbor_idx = *group_it++;
      uint8_t data_size = *group_it++;
      
      if(neighbor_idx < pre_voxel_ptr_size)
      {
        OctVoxType* voxel_ptr = top_voxels_2_search[neighbor_idx];
        if (voxel_ptr) {
          while (data_size--) {
            uint8_t _local_idx = (*group_it++)^local_idx;
            if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
              const float dist2 = (__sub_point - point).squaredNorm();
              top_K.try_insert(dist2, __sub_point);
            }
          }
        }
        else group_it+=data_size;
        continue;
      }

      KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
      const KEY n_key = key + delta_key;

      if (auto iter = grids_.find(n_key); iter != grids_.end()){
        OctVoxType* voxel_ptr = &iter->second->second;
        while (data_size--){
          const uint8_t _local_idx = (*group_it++)^local_idx;
          if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
            float dist2 = (__sub_point - point).squaredNorm();
            top_K.try_insert(dist2, __sub_point);
          }
        }
      }
      else group_it+=data_size;
    }

    if (top_K.count == 5)
      if (top_K.max_dist2_ < orders_min_dis2_scaled_[group_idx]){
        break;
      }

  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::clear() {
  grids_.clear();
  data_.clear();
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::printInfo() const {
    std::cout << " ---> OctVoxMap info. Size: " << data_.size() 
              << " Capacity: " << capacity_ << std::endl;
}

}

#endif