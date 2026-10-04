#pragma once
#include <QWidget>
#include <memory>

class QNetworkAccessManager;
class QNetworkReply;
class QTimer;
class QLabel;

namespace rcj {

// Polls a single-frame JPEG endpoint from web_video_server (e.g.
//   http://192.168.149.1:8080/stream?topic=/depth_cam/rgb/image_raw).
// For true multipart/x-mixed-replace you'd subclass QThread and parse the
// stream — left for P+. For most mentorpi setups the polling mode is fine.
class MjpegStream : public QWidget {
  Q_OBJECT
 public:
  explicit MjpegStream(QWidget* parent = nullptr);
  ~MjpegStream() override;
  void setUrl(const QString& url);
  // 由 ROS 压缩图话题喂图(优先于 HTTP 轮询)
public slots:
  void setImage(const QImage& img);

 private:
  struct Private;
  std::unique_ptr<Private> d_;
};

}  // namespace rcj
