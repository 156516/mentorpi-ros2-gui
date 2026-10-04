// SPDX-License-Identifier: MIT
#pragma once
//
// map_editor.h — an editable copy of a nav_msgs/OccupancyGrid plus PGM/YAML
// (de)serialisation. Deliberately pure logic (nav_msgs + the C++ stdlib only,
// no Qt widgets, no rclcpp) so it can be unit-tested standalone, mirroring
// ros_to_qt.h.
//
// Cell values follow the nav_msgs convention: 0 = free, 100 = occupied,
// -1 = unknown.
//
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace rcj {

class MapEditBuffer {
 public:
  struct Patch { uint32_t idx; int8_t old_v; int8_t new_v; };

  void reset(const nav_msgs::msg::OccupancyGrid& g) {
    grid_ = g;
    undo_.clear();
    redo_.clear();
    patch_.clear();
  }

  bool empty() const { return grid_.data.empty(); }
  const nav_msgs::msg::OccupancyGrid& grid() const { return grid_; }

  // Paint a filled disk of radius_m centred at world (wx, wy). Honours the
  // grid's origin orientation (yaw). `value` is the new cell value.
  void applyBrush(double wx, double wy, double radius_m, int8_t value) {
    const auto& info = grid_.info;
    const int w = static_cast<int>(info.width);
    const int h = static_cast<int>(info.height);
    const double res = info.resolution;
    if (w <= 0 || h <= 0 || res <= 0.0) return;
    if (static_cast<int>(grid_.data.size()) < w * h) return;

    const double qz = info.origin.orientation.z;
    const double qw = info.origin.orientation.w;
    const double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    // world → map frame (undo the origin translation + yaw)
    const double dx = wx - info.origin.position.x;
    const double dy = wy - info.origin.position.y;
    const double lx =  cy * dx + sy * dy;
    const double ly = -sy * dx + cy * dy;
    const double ccx = lx / res;   // continuous column
    const double ccy = ly / res;   // continuous row (row 0 = lowest y)
    const double rc  = radius_m / res;
    const double r2  = rc * rc;

    const int c0 = std::max(0, static_cast<int>(std::floor(ccx - rc)));
    const int c1 = std::min(w - 1, static_cast<int>(std::floor(ccx + rc)));
    const int r0 = std::max(0, static_cast<int>(std::floor(ccy - rc)));
    const int r1 = std::min(h - 1, static_cast<int>(std::floor(ccy + rc)));
    for (int r = r0; r <= r1; ++r) {
      for (int c = c0; c <= c1; ++c) {
        const double ddx = (c + 0.5) - ccx;
        const double ddy = (r + 0.5) - ccy;
        if (ddx * ddx + ddy * ddy > r2) continue;
        const uint32_t idx = static_cast<uint32_t>(r) * w + c;
        const int8_t old_v = grid_.data[idx];
        if (old_v == value) continue;
        patch_.push_back({idx, old_v, value});
        grid_.data[idx] = value;
      }
    }
  }

  // A stroke groups all the brush applications between begin/end into ONE
  // undo step. Re-touching an already-painted cell within the stroke is a
  // no-op (applyBrush skips cells already equal to `value`).
  void beginStroke() { patch_.clear(); }
  void endStroke() {
    if (!patch_.empty()) {
      undo_.push_back(std::move(patch_));
      patch_.clear();
      redo_.clear();
      if (undo_.size() > kMaxUndo) undo_.erase(undo_.begin());
    }
  }

  bool canUndo() const { return !undo_.empty(); }
  bool canRedo() const { return !redo_.empty(); }

  bool undo() {
    if (undo_.empty()) return false;
    std::vector<Patch> p = std::move(undo_.back());
    undo_.pop_back();
    // Reverse order so a cell touched twice ends on its ORIGINAL value.
    for (auto it = p.rbegin(); it != p.rend(); ++it) grid_.data[it->idx] = it->old_v;
    redo_.push_back(std::move(p));
    return true;
  }

  bool redo() {
    if (redo_.empty()) return false;
    std::vector<Patch> p = std::move(redo_.back());
    redo_.pop_back();
    for (const auto& e : p) grid_.data[e.idx] = e.new_v;
    undo_.push_back(std::move(p));
    return true;
  }

  void clearHistory() { undo_.clear(); redo_.clear(); patch_.clear(); }

 private:
  static constexpr size_t kMaxUndo = 100;
  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<Patch> patch_;
  std::vector<std::vector<Patch>> undo_, redo_;
};

// ---- PGM / YAML serialisation (map_server convention) --------------------
//
// Pixel value map used on BOTH save and load:
//   occupied(100) <-> 0     free(0) <-> 254     unknown(-1) <-> 205
inline uint8_t occToPixel(int8_t v) {
  if (v < 0) return 205;
  if (v >= 50) return 0;
  return 254;
}
inline int8_t pixelToOcc(int pixel, double occupied_thresh, double free_thresh) {
  // map_server: occ = (255 - pixel) / 255  (negate == 0)
  const double occ = (255.0 - static_cast<double>(pixel)) / 255.0;
  if (occ > occupied_thresh) return 100;
  if (occ < free_thresh)     return 0;
  return -1;
}

// nav_msgs/OccupancyGrid -> binary PGM (P5). Rows are flipped: PGM row 0 is
// the TOP of the map, while OccupancyGrid row 0 is the BOTTOM.
inline std::vector<uint8_t> gridToPgm(const nav_msgs::msg::OccupancyGrid& g) {
  const int w = static_cast<int>(g.info.width);
  const int h = static_cast<int>(g.info.height);
  std::vector<uint8_t> out;
  if (w <= 0 || h <= 0) return out;
  char hdr[64];
  const int n = std::snprintf(hdr, sizeof(hdr), "P5\n%d %d\n255\n", w, h);
  out.insert(out.end(), hdr, hdr + n);
  out.resize(out.size() + static_cast<size_t>(w) * h);
  uint8_t* px = out.data() + out.size() - static_cast<size_t>(w) * h;
  for (int row = 0; row < h; ++row) {
    const int src_row = h - 1 - row;               // flip
    for (int c = 0; c < w; ++c) {
      px[row * w + c] = occToPixel(g.data[static_cast<size_t>(src_row) * w + c]);
    }
  }
  return out;
}

inline std::string gridToYaml(const nav_msgs::msg::OccupancyGrid& g,
                              const std::string& image_name) {
  const auto& o = g.info.origin;
  const double qz = o.orientation.z, qw = o.orientation.w;
  const double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "image: %s\n"
                "resolution: %.6f\n"
                "origin: [%.6f, %.6f, %.6f]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n",
                image_name.c_str(), g.info.resolution,
                o.position.x, o.position.y, yaw);
  return std::string(buf);
}

// Parse a minimal map_server YAML. Only the keys we write are understood;
// unknown keys are ignored. Returns false + err on a missing required key.
inline bool parseMapYaml(const std::string& yaml,
                         double& resolution, double& ox, double& oy, double& oyaw,
                         double& occupied_thresh, double& free_thresh,
                         std::string& image_name, std::string* err) {
  image_name = "map.pgm";
  resolution = 0.05; ox = 0.0; oy = 0.0; oyaw = 0.0;
  occupied_thresh = 0.65; free_thresh = 0.196;
  bool have_res = false;

  size_t pos = 0;
  while (pos < yaml.size()) {
    size_t eol = yaml.find('\n', pos);
    if (eol == std::string::npos) eol = yaml.size();
    std::string line = yaml.substr(pos, eol - pos);
    pos = eol + 1;
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    auto trim = [](std::string s) {
      size_t a = s.find_first_not_of(" \t\r\"'");
      size_t b = s.find_last_not_of(" \t\r\"'");
      return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
    };
    key = trim(key); val = trim(val);
    if (key == "image")            image_name = val;
    else if (key == "resolution") { resolution = std::atof(val.c_str()); have_res = true; }
    else if (key == "occupied_thresh") occupied_thresh = std::atof(val.c_str());
    else if (key == "free_thresh")     free_thresh     = std::atof(val.c_str());
    else if (key == "origin") {
      // origin: [x, y, yaw]
      const size_t lb = val.find('[');
      const size_t rb = val.find(']');
      if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        const std::string inner = val.substr(lb + 1, rb - lb - 1);
        double v[3] = {0, 0, 0};
        int k = 0;
        size_t p = 0;
        while (k < 3 && p < inner.size()) {
          size_t comma = inner.find(',', p);
          if (comma == std::string::npos) comma = inner.size();
          v[k++] = std::atof(inner.substr(p, comma - p).c_str());
          p = comma + 1;
        }
        ox = v[0]; oy = v[1]; oyaw = v[2];
      }
    }
  }
  if (!have_res) {
    if (err) *err = "YAML missing 'resolution'";
    return false;
  }
  if (resolution <= 0.0) {
    if (err) *err = "YAML resolution must be > 0";
    return false;
  }
  return true;
}

// Parse P5 PGM bytes (skipping '#' comments) into width/height/pixels.
inline bool parsePgmP5(const std::vector<uint8_t>& d,
                       int& w, int& h, const uint8_t*& pixels, std::string* err) {
  size_t i = 0;
  auto skip_ws_comments = [&](void) {
    for (;;) {
      while (i < d.size() && (d[i] == ' ' || d[i] == '\t' || d[i] == '\r' || d[i] == '\n')) ++i;
      if (i < d.size() && d[i] == '#') {
        while (i < d.size() && d[i] != '\n') ++i;
      } else break;
    }
  };
  if (d.size() < 2 || d[0] != 'P' || d[1] != '5') {
    if (err) *err = "not a binary PGM (P5)";
    return false;
  }
  i = 2;
  skip_ws_comments();
  int vals[3] = {0, 0, 0};
  for (int k = 0; k < 3; ++k) {
    skip_ws_comments();
    if (i >= d.size() || d[i] < '0' || d[i] > '9') {
      if (err) *err = "malformed PGM header";
      return false;
    }
    long v = 0;
    while (i < d.size() && d[i] >= '0' && d[i] <= '9') { v = v * 10 + (d[i] - '0'); ++i; }
    vals[k] = static_cast<int>(v);
  }
  if (vals[2] != 255) {
    if (err) *err = "only 8-bit PGM (maxval 255) is supported";
    return false;
  }
  if (i >= d.size()) { if (err) *err = "PGM truncated after header"; return false; }
  ++i;  // single whitespace after maxval
  w = vals[0]; h = vals[1];
  if (w <= 0 || h <= 0) { if (err) *err = "PGM has non-positive size"; return false; }
  const size_t need = static_cast<size_t>(w) * h;
  if (d.size() - i < need) { if (err) *err = "PGM pixel data truncated"; return false; }
  pixels = d.data() + i;
  return true;
}

// YAML text + PGM bytes -> nav_msgs/OccupancyGrid.
inline bool loadPgmYaml(const std::string& yaml_text,
                        const std::vector<uint8_t>& pgm_bytes,
                        nav_msgs::msg::OccupancyGrid& out,
                        std::string* err) {
  double res = 0, ox = 0, oy = 0, oyaw = 0, occ_t = 0.65, free_t = 0.196;
  std::string image_name;
  if (!parseMapYaml(yaml_text, res, ox, oy, oyaw, occ_t, free_t, image_name, err))
    return false;
  int w = 0, h = 0;
  const uint8_t* px = nullptr;
  if (!parsePgmP5(pgm_bytes, w, h, px, err)) return false;

  out = nav_msgs::msg::OccupancyGrid{};
  out.info.resolution = res;
  out.info.width  = static_cast<uint32_t>(w);
  out.info.height = static_cast<uint32_t>(h);
  out.info.origin.position.x = ox;
  out.info.origin.position.y = oy;
  out.info.origin.position.z = 0.0;
  out.info.origin.orientation.x = 0.0;
  out.info.origin.orientation.y = 0.0;
  out.info.origin.orientation.z = std::sin(oyaw / 2.0);
  out.info.origin.orientation.w = std::cos(oyaw / 2.0);
  out.data.resize(static_cast<size_t>(w) * h);
  for (int row = 0; row < h; ++row) {
    const int dst_row = h - 1 - row;              // flip back to ROS order
    for (int c = 0; c < w; ++c) {
      out.data[static_cast<size_t>(dst_row) * w + c] =
          pixelToOcc(px[static_cast<size_t>(row) * w + c], occ_t, free_t);
    }
  }
  return true;
}

}  // namespace rcj
