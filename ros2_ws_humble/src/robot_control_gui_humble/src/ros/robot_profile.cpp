// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ros/robot_profile.h"

#include <QDir>
#include <QFile>
#include <QSettings>
#include <QTextStream>

namespace rcj {

// 必须共用一个实例:分别写 static 会变成两个不同的对象(踩过)
static RobotProfile& theProfile() {
  static RobotProfile p;
  return p;
}
const RobotProfile& RobotProfile::cur()         { return theProfile(); }
RobotProfile&       RobotProfile::mutableCur()  { return theProfile(); }

bool RobotProfile::loadFile(const QString& path, RobotProfile& out, QString* err) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (err) *err = "打不开 " + path;
    return false;
  }
  out = RobotProfile{};                       // 先回到默认
  out.file = QFileInfo(path).fileName();
  QTextStream in(&f);
  auto val = [](QString s) {                  // 去掉引号/空白
    s = s.trimmed();
    if (s.size() >= 2 && (s.startsWith('"') || s.startsWith('\'')) && s.endsWith(s[0]))
      s = s.mid(1, s.size() - 2);
    return s.trimmed();
  };
  auto getbool = [&](const QString& v) { return v.compare("true", Qt::CaseInsensitive) == 0 || v == "1"; };
  while (!in.atEnd()) {
    QString line = in.readLine();
    const int hash = line.indexOf('#');       // 去注释
    if (hash >= 0) line = line.left(hash);
    const int colon = line.indexOf(':');
    if (colon < 0) continue;
    const QString k = line.left(colon).trimmed();
    const QString v = val(line.mid(colon + 1));
    if (v.isEmpty()) continue;
    if      (k == "name")            out.name = v;
    else if (k == "robot_ip")        out.robot_ip = v;
    else if (k == "cmd_vel")         out.cmd_vel = v;
    else if (k == "scan")            out.scan = v;
    else if (k == "odom")            out.odom = v;
    else if (k == "map")             out.map = v;
    else if (k == "plan")            out.plan = v;
    else if (k == "battery")         out.battery = v;
    else if (k == "battery_type")    out.battery_type = v;
    else if (k == "camera_topic")    out.camera_topic = v;
    else if (k == "map_frame")       out.map_frame = v;
    else if (k == "odom_frame")      out.odom_frame = v;
    else if (k == "base_frame")      out.base_frame = v;
    else if (k == "lidar_frame")     out.lidar_frame = v;
    else if (k == "has_nav2")            out.has_nav2 = getbool(v);
    else if (k == "has_slam_controller") out.has_slam_controller = getbool(v);
    else if (k == "has_camera")          out.has_camera = getbool(v);
  }
  return true;
}

QStringList RobotProfile::listProfiles(const QString& dir) {
  QStringList out;
  QDir d(dir);
  if (!d.exists()) return out;
  for (const QString& f : d.entryList({"*.yaml", "*.yml"}, QDir::Files, QDir::Name)) {
    if (f.startsWith('_')) continue;          // _template.yaml 不算
    out << f;
  }
  return out;
}

void RobotProfile::loadSelected(const QString& dir) {
  QSettings s;
  const QString want = s.value("robot/profile", "mentorpi.yaml").toString();
  const QString path = dir + "/" + want;
  RobotProfile p;
  QString err;
  if (loadFile(path, p, &err)) {
    mutableCur() = p;
  } else {
    mutableCur() = RobotProfile{};             // 回默认(mentorpi)
  }
}

}  // namespace rcj
