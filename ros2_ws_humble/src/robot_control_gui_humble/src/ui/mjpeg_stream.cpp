// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/mjpeg_stream.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QPainter>
#include <QImageReader>
#include <QBuffer>

namespace rcj {

// All internals in this TU so unique_ptr<Private> works without
// exposing the struct in the header.
struct MjpegStream::Private {
  QNetworkAccessManager nam;
  QNetworkReply* reply{nullptr};
  QTimer*        poll_timer{nullptr};
  QLabel*        view{nullptr};
  QString        url;
};

MjpegStream::~MjpegStream() = default;

MjpegStream::MjpegStream(QWidget* parent)
    : QWidget(parent), d_(std::make_unique<Private>()) {
  auto* l = new QVBoxLayout(this);
  l->setContentsMargins(0, 0, 0, 0);

  d_->view = new QLabel(this);
  // 最小宽度别太大:控制页是 摇杆160 + 仪表180 + 摄像头 并排,摄像头这一项
  // 直接决定该页的最小宽度。320 时整页 762px,刚好超出页签区视口(760)导致
  // 一直挂着横向滚动条;280 留出余量。
  d_->view->setMinimumSize(280, 210);
  d_->view->setAlignment(Qt::AlignCenter);
  d_->view->setStyleSheet("background:#222;color:#aaa;");
  d_->view->setText(tr("(未连接摄像头)"));

  l->addWidget(d_->view);

  d_->poll_timer = new QTimer(this);
  d_->poll_timer->setInterval(200);   // 5 fps for JPEG polling
  connect(d_->poll_timer, &QTimer::timeout, this, [this]() {
    if (d_->url.isEmpty() || d_->reply) return;
    QNetworkRequest req{QUrl(d_->url)};
    req.setRawHeader("User-Agent", "robot_control_gui_humble");
    d_->reply = d_->nam.get(req);
    connect(d_->reply, &QNetworkReply::finished, this, [this]() {
      if (!d_->reply) return;
      if (d_->reply->error() == QNetworkReply::NoError) {
        const auto bytes = d_->reply->readAll();
        QImage img;
        if (img.loadFromData(bytes)) {
          d_->view->setPixmap(QPixmap::fromImage(img).scaled(
              d_->view->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
      }
      d_->reply->deleteLater();
      d_->reply = nullptr;
    });
  });
}

void MjpegStream::setImage(const QImage& img) {
  if (img.isNull() || !d_->view) return;
  d_->view->setText(QString());          // 清掉"(未连接摄像头)"
  d_->view->setPixmap(QPixmap::fromImage(img).scaled(
      d_->view->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MjpegStream::setUrl(const QString& url) {
  if (url == d_->url) return;
  d_->url = url;
  if (d_->reply) {
    d_->reply->abort();
    d_->reply->deleteLater();
    d_->reply = nullptr;
  }
  if (url.isEmpty()) {
    d_->poll_timer->stop();
    d_->view->setText(tr("(未连接摄像头)"));
  } else {
    d_->poll_timer->start();
  }
}

}  // namespace rcj
