// SPDX-License-Identifier: MIT
//
// Pure-C++ tests for MapEditBuffer + PGM/YAML serialisation (no rclcpp).
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "robot_control_gui_humble/ros/map_editor.h"

using rcj::MapEditBuffer;
using rcj::gridToPgm;
using rcj::gridToYaml;
using rcj::loadPgmYaml;

namespace {

nav_msgs::msg::OccupancyGrid makeGrid(int w, int h, double res,
                                      double ox, double oy, double yaw = 0.0) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = static_cast<uint32_t>(w);
  g.info.height = static_cast<uint32_t>(h);
  g.info.resolution = res;
  g.info.origin.position.x = ox;
  g.info.origin.position.y = oy;
  g.info.origin.orientation.z = std::sin(yaw / 2.0);
  g.info.origin.orientation.w = std::cos(yaw / 2.0);
  g.data.assign(static_cast<size_t>(w) * h, 0);   // all free
  return g;
}

}  // namespace

TEST(MapEditBuffer, BrushWritesCellsAroundWorldPoint) {
  MapEditBuffer b;
  b.reset(makeGrid(10, 10, 0.1, 0.0, 0.0));   // 1m x 1m, origin at 0,0
  b.beginStroke();
  b.applyBrush(0.55, 0.55, 0.06, 100);        // ~1 cell radius at (5,5)
  b.endStroke();
  // cell (col 5, row 5) must be occupied
  EXPECT_EQ(b.grid().data[5 * 10 + 5], 100);
  // a far corner must be untouched
  EXPECT_EQ(b.grid().data[0], 0);
}

TEST(MapEditBuffer, BrushHonoursOriginTranslation) {
  MapEditBuffer b;
  b.reset(makeGrid(10, 10, 0.1, -5.0, -5.0));  // origin at (-5,-5); spans [-5,-4]^2
  b.beginStroke();
  b.applyBrush(-4.45, -4.45, 0.06, 100);       // local (0.55,0.55) -> cell (5,5) centre
  b.endStroke();
  EXPECT_EQ(b.grid().data[5 * 10 + 5], 100);
}

TEST(MapEditBuffer, BrushHonoursOriginYaw) {
  MapEditBuffer b;
  const double yaw = M_PI / 2.0;               // origin rotated 90 deg
  b.reset(makeGrid(10, 10, 0.1, 0.0, 0.0, yaw));
  // world point (0, 1) is, in the grid's own frame, local (1, 0)
  // (rotating world by -yaw): lx = cos(-90)*0 + sin(-90)*1 = -1 ... use the
  // inverse mapping consistently: local = R(-yaw) * world.
  b.beginStroke();
  b.applyBrush(0.0, 0.55, 0.06, 100);
  b.endStroke();
  // local (0.55, 0) -> col 5, row 0
  EXPECT_EQ(b.grid().data[0 * 10 + 5], 100);
}

TEST(MapEditBuffer, UndoRedoSingleStroke) {
  MapEditBuffer b;
  b.reset(makeGrid(4, 4, 0.1, 0.0, 0.0));
  b.beginStroke();
  b.applyBrush(0.15, 0.15, 0.06, 100);     // local (0.15,0.15) -> cell (col1,row1) = index 5
  b.endStroke();
  EXPECT_EQ(b.grid().data[5], 100);

  EXPECT_TRUE(b.canUndo());
  EXPECT_TRUE(b.undo());
  EXPECT_EQ(b.grid().data[5], 0);          // restored
  EXPECT_TRUE(b.canRedo());
  EXPECT_TRUE(b.redo());
  EXPECT_EQ(b.grid().data[5], 100);        // re-applied
}

TEST(MapEditBuffer, UndoRestoresOriginalWhenCellTouchedTwice) {
  MapEditBuffer b;
  b.reset(makeGrid(4, 4, 0.1, 0.0, 0.0));
  b.beginStroke();
  b.applyBrush(0.15, 0.15, 0.06, 100);     // 0 -> 100
  b.applyBrush(0.15, 0.15, 0.06, 0);       // 100 -> 0 (erase again)
  b.endStroke();
  EXPECT_EQ(b.grid().data[5], 0);
  EXPECT_TRUE(b.undo());
  EXPECT_EQ(b.grid().data[5], 0);          // back to the ORIGINAL value (0)
}

TEST(MapEditBuffer, UndoDepthCappedButWorks) {
  MapEditBuffer b;
  b.reset(makeGrid(4, 4, 0.1, 0.0, 0.0));
  for (int i = 0; i < 150; ++i) {
    b.beginStroke();
    b.applyBrush(0.15, 0.15, 0.06, (i % 2 == 0) ? 100 : 0);
    b.endStroke();
  }
  EXPECT_TRUE(b.canUndo());   // never empty
  EXPECT_TRUE(b.undo());
}

TEST(PgmYaml, RoundTripPreservesCellsAndGeometry) {
  nav_msgs::msg::OccupancyGrid g = makeGrid(5, 4, 0.05, -1.0, -2.0);
  g.data[0] = 100;                       // bottom-left occupied
  g.data[4] = -1;                        // bottom-right unknown
  g.data[static_cast<size_t>(3) * 5 + 0] = 100;  // top-left occupied

  const auto pgm  = gridToPgm(g);
  const auto yaml = gridToYaml(g, "map.pgm");
  ASSERT_FALSE(pgm.empty());
  // P5 header
  std::string head(reinterpret_cast<const char*>(pgm.data()), 12);
  EXPECT_EQ(head.rfind("P5", 0), 0u);

  nav_msgs::msg::OccupancyGrid out;
  std::string err;
  ASSERT_TRUE(loadPgmYaml(yaml, pgm, out, &err)) << err;
  EXPECT_EQ(out.info.width, 5u);
  EXPECT_EQ(out.info.height, 4u);
  EXPECT_NEAR(out.info.resolution, 0.05, 1e-9);
  EXPECT_NEAR(out.info.origin.position.x, -1.0, 1e-6);
  EXPECT_NEAR(out.info.origin.position.y, -2.0, 1e-6);
  ASSERT_EQ(out.data.size(), g.data.size());
  for (size_t i = 0; i < g.data.size(); ++i) {
    // -1 and 100 survive; free 0 survives
    EXPECT_EQ(out.data[i], g.data[i]) << "cell " << i;
  }
}

TEST(PgmYaml, RejectsNonPgmInput) {
  nav_msgs::msg::OccupancyGrid out;
  std::string err;
  const std::vector<uint8_t> junk = {'N', 'O', 'P', 'E'};
  EXPECT_FALSE(loadPgmYaml("resolution: 0.05\n", junk, out, &err));
  EXPECT_FALSE(err.empty());
}

TEST(PgmYaml, RejectsYamlWithoutResolution) {
  nav_msgs::msg::OccupancyGrid out;
  std::string err;
  std::vector<uint8_t> pgm = {'P', '5', '\n', '1', ' ', '1', '\n', '2', '5', '5', '\n', 0};
  EXPECT_FALSE(loadPgmYaml("image: x.pgm\n", pgm, out, &err));
}

TEST(PgmYaml, ParsesOriginWithYawAndTrailingComment) {
  nav_msgs::msg::OccupancyGrid g = makeGrid(2, 2, 0.1, 1.0, 2.0, 0.5);
  const auto pgm  = gridToPgm(g);
  const auto yaml = gridToYaml(g, "m.pgm") + "# trailing comment\n";
  nav_msgs::msg::OccupancyGrid out;
  std::string err;
  ASSERT_TRUE(loadPgmYaml(yaml, pgm, out, &err)) << err;
  const double yaw = std::atan2(2.0 * out.info.origin.orientation.w * out.info.origin.orientation.z,
                                1.0 - 2.0 * out.info.origin.orientation.z * out.info.origin.orientation.z);
  EXPECT_NEAR(yaw, 0.5, 1e-6);
}
