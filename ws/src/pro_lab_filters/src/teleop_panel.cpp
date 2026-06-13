#include "pro_lab_filters/teleop_panel.hpp"

#include <cmath>

#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QEvent>
#include <QFont>
#include <QKeyEvent>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPlainTextEdit>
#include <QSpinBox>
#include <QString>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>

namespace pro_lab_rviz {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kGapPx   = 11.0;      // visible gap between wedges, in pixels
constexpr double kCenterFrac = 0.30;   // central stop disc radius fraction
constexpr double kInnerFrac  = 0.48;   // wedge inner radius fraction
constexpr double kOuterFrac  = 0.85;   // wedge outer radius fraction

QColor wedgeColor(bool pressed) {
  return pressed ? QColor("#4078f2") : QColor("#2a2d36");
}

QColor stopColor(bool pressed) {
  return pressed ? QColor("#e64545") : QColor("#3a3d46");
}

// Build a pie-slice path with a *constant pixel-width* gap on each side
// (so the gap looks uniform from inner to outer). To do this, the gap's
// half-angle differs at inner vs outer radius: for a target pixel gap g,
// the half-angle at radius r is g/(2r) rad. The radial-ish sides of the
// wedge then aren't perfectly radial - they're a chord between the inner
// and outer arc endpoints, which is what makes the gap appear parallel.
QPainterPath wedgePath(QPointF c, double r_in, double r_out,
                       int center_deg, int span_deg) {
  const double half_deg     = span_deg / 2.0;
  const double gap_out_half = (kGapPx / (2.0 * r_out)) * kRad2Deg;
  const double gap_in_half  = (kGapPx / (2.0 * r_in))  * kRad2Deg;

  const double a0_out = center_deg - half_deg + gap_out_half;
  const double a1_out = center_deg + half_deg - gap_out_half;
  const double a0_in  = center_deg - half_deg + gap_in_half;
  const double a1_in  = center_deg + half_deg - gap_in_half;

  const QRectF outerRect(c.x() - r_out, c.y() - r_out, 2 * r_out, 2 * r_out);
  const QRectF innerRect(c.x() - r_in,  c.y() - r_in,  2 * r_in,  2 * r_in);

  QPainterPath p;
  p.arcMoveTo(outerRect, a0_out);                        // outer @ a0_out
  p.arcTo(outerRect, a0_out, a1_out - a0_out);           // outer arc CCW
  p.arcTo(innerRect, a1_in,  a0_in  - a1_in);            // implicit chord in,
                                                         // then inner arc CW
  p.closeSubpath();                                      // chord back to start
  return p;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────
//  TeleopPad
// ─────────────────────────────────────────────────────────────────────────

TeleopPad::TeleopPad(QWidget * parent) : QWidget(parent) {
  setMinimumSize(180, 180);
  setMouseTracking(true);
  setFocusPolicy(Qt::NoFocus);
}

TeleopPad::Sector TeleopPad::hitTest(const QPoint & p) const {
  const int s = std::min(width(), height());
  const QPointF c(width() / 2.0, height() / 2.0);
  const double dx = p.x() - c.x();
  const double dy = c.y() - p.y();   // flip Y so +dy is up
  const double r  = std::hypot(dx, dy);
  if (r < kCenterFrac * s / 2.0) return STOP;
  if (r < kInnerFrac  * s / 2.0) return NONE;     // dead band
  if (r > kOuterFrac  * s / 2.0) return NONE;
  double a = std::atan2(dy, dx) * 180.0 / M_PI;   // -180..180
  // 4 wedges centred at 90 (up), 0 (right), -90 (down), 180/-180 (left).
  if (a >  45 && a <= 135) return UP;
  if (a > -45 && a <=  45) return RIGHT;
  if (a > -135 && a <= -45) return DOWN;
  return LEFT;
}

void TeleopPad::setSector(Sector s) {
  if (s == sector_) {
    return;
  }
  sector_ = s;
  update();
  if (onSectorChanged) {
    onSectorChanged(sector_);
  }
}

void TeleopPad::mousePressEvent(QMouseEvent * e) {
  if (e->button() == Qt::LeftButton) {
    setSector(hitTest(e->pos()));
  }
}

void TeleopPad::mouseReleaseEvent(QMouseEvent *) {
  setSector(NONE);
}

void TeleopPad::mouseMoveEvent(QMouseEvent * e) {
  // While left button is held, dragging between sectors switches direction;
  // dragging into the dead band stops the robot.
  if (e->buttons() & Qt::LeftButton) {
    setSector(hitTest(e->pos()));
  }
}

void TeleopPad::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor("#1a1b1f"));

  const int    s     = std::min(width(), height());
  const QPointF c    (width() / 2.0, height() / 2.0);
  const double r_in  = kInnerFrac * s / 2.0;
  const double r_out = kOuterFrac * s / 2.0;
  const double r_stop = kCenterFrac * s / 2.0;

  struct W { int center_deg; Sector sec; };
  const W wedges[4] = {
    { 90,   UP    },
    {  0,   RIGHT },
    {-90,   DOWN  },
    { 180,  LEFT  },
  };

  // Draw triangular arrow glyphs ourselves so they sit pixel-perfect at
  // the wedge centre - Qt's ▲/▼ font glyphs have asymmetric metrics that
  // make AlignCenter look slightly off, especially for ■.
  const double rmid    = (r_in + r_out) / 2.0;
  const double tri_h   = s * 0.08;     // arrow size
  const double tri_w   = s * 0.10;
  p.setPen(Qt::NoPen);

  for (const auto & w : wedges) {
    QPainterPath path = wedgePath(c, r_in, r_out, w.center_deg, 90);
    p.fillPath(path, wedgeColor(sector_ == w.sec));

    const double a = w.center_deg * kDeg2Rad;
    const QPointF tip(c.x() + rmid * std::cos(a),
                      c.y() - rmid * std::sin(a));
    // Triangle tip points outward (away from centre). Rotate a base triangle
    // so it points along the wedge's angle.
    const double cs = std::cos(a), sn = std::sin(a);
    auto rot = [&](double dx, double dy) {
      // dx is along outward radial, dy is tangential
      return QPointF(tip.x() + dx * cs - dy * (-sn),
                     tip.y() - dx * sn - dy *   cs);
    };
    QPainterPath arrow;
    arrow.moveTo(rot( tri_h / 2.0,        0.0));
    arrow.lineTo(rot(-tri_h / 2.0,  tri_w / 2.0));
    arrow.lineTo(rot(-tri_h / 2.0, -tri_w / 2.0));
    arrow.closeSubpath();
    p.fillPath(arrow, QColor("#e8e8e8"));
  }

  // Central stop disc + filled square (drawn, not glyph, so it's pixel-centred).
  p.setBrush(stopColor(sector_ == STOP));
  p.drawEllipse(c, r_stop, r_stop);
  const double sq = r_stop * 0.32;
  QRectF squareRect(c.x() - sq, c.y() - sq, 2 * sq, 2 * sq);
  p.fillRect(squareRect, QColor("#e8e8e8"));
}

// ─────────────────────────────────────────────────────────────────────────
//  TeleopPanel
// ─────────────────────────────────────────────────────────────────────────

TeleopPanel::TeleopPanel(QWidget * parent) : rviz_common::Panel(parent) {
  pad_ = new TeleopPad(this);
  pad_->onSectorChanged = [this](TeleopPad::Sector s) { onSector(s); };

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(pad_);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TeleopPanel::publishTick);
}

TeleopPanel::~TeleopPanel() {
  if (qApp) {
    qApp->removeEventFilter(this);
  }
}

void TeleopPanel::onInitialize() {
  // Self-owned rclcpp node - borrowing RViz's shared node via
  // getRosNodeAbstraction() races with RViz init on some Jazzy builds and
  // silently leaves the publisher unbound. A private node is reliable and
  // publishers don't need a running executor to send messages.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node_ = std::make_shared<rclcpp::Node>("rviz_teleop_panel");
  pub_  = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_in", rclcpp::QoS(10));
  timer_->start(50);  // 20 Hz

  // Global key handler: works no matter which RViz panel has focus. We
  // install on qApp instead of binding QShortcut so the key-release event
  // is observable (releasing the arrow stops the robot, matching the
  // dpad's press-and-hold semantics).
  if (qApp) {
    qApp->installEventFilter(this);
  }
}

namespace {
// Skip the global key filter when the user is typing into a text input -
// otherwise arrow keys would steal focus from line edits / spin boxes /
// combos in RViz config dialogs.
bool focusIsTextInput() {
  QWidget * f = QApplication::focusWidget();
  if (!f) {
    return false;
  }
  if (qobject_cast<QLineEdit *>(f) || qobject_cast<QTextEdit *>(f) ||
      qobject_cast<QPlainTextEdit *>(f) || qobject_cast<QComboBox *>(f) ||
      qobject_cast<QSpinBox *>(f) || qobject_cast<QDoubleSpinBox *>(f)) {
    return true;
  }
  // Catch QAbstractSpinBox / inline editors that Qt nests inside item views.
  if (f->inherits("QAbstractSpinBox") || f->inherits("QAbstractItemView")) {
    return true;
  }
  return false;
}
}  // namespace

bool TeleopPanel::eventFilter(QObject * /*obj*/, QEvent * ev) {
  if (ev->type() != QEvent::KeyPress && ev->type() != QEvent::KeyRelease) {
    return false;
  }
  auto * ke = static_cast<QKeyEvent *>(ev);
  // Auto-repeat fires KeyPress over and over while holding - we already
  // set the sector on the first press, so swallow the rest silently
  // (returning false lets RViz still see them, which is fine).
  if (ke->isAutoRepeat()) {
    return false;
  }
  if (focusIsTextInput()) {
    return false;
  }
  const int k = ke->key();
  const bool press = (ev->type() == QEvent::KeyPress);

  // Multiple key sets: arrow keys, WASD (gaming-style, S now = backward),
  // IJKL (right-hand ROS teleop). Diagonals (Q/E/Y/C and U/O/M/.) feed v+w
  // directly for arc motion instead of pure rotation. X / Space = stop.
  TeleopPad::Sector s = TeleopPad::NONE;
  bool use_sector = true;
  double v_arc = 0.0, w_arc = 0.0;
  switch (k) {
    // forward
    case Qt::Key_Up:    case Qt::Key_W: case Qt::Key_I:
      s = TeleopPad::UP; break;
    // backward
    case Qt::Key_Down:  case Qt::Key_S: case Qt::Key_K: case Qt::Key_Comma:
      s = TeleopPad::DOWN; break;
    // left (pure rotation in place)
    case Qt::Key_Left:  case Qt::Key_A: case Qt::Key_J:
      s = TeleopPad::LEFT; break;
    // right (pure rotation in place)
    case Qt::Key_Right: case Qt::Key_D: case Qt::Key_L:
      s = TeleopPad::RIGHT; break;
    // stop
    case Qt::Key_Space: case Qt::Key_X:
      s = TeleopPad::STOP; break;
    // diagonals: forward-left arc
    case Qt::Key_Q: case Qt::Key_U:
      use_sector = false; v_arc =  v_max_; w_arc =  w_max_ * 0.6; break;
    // forward-right arc
    case Qt::Key_E: case Qt::Key_O:
      use_sector = false; v_arc =  v_max_; w_arc = -w_max_ * 0.6; break;
    // backward-left arc
    case Qt::Key_Y: case Qt::Key_M:
      use_sector = false; v_arc = -v_max_; w_arc =  w_max_ * 0.6; break;
    // backward-right arc
    case Qt::Key_C: case Qt::Key_Period:
      use_sector = false; v_arc = -v_max_; w_arc = -w_max_ * 0.6; break;
    default: return false;
  }
  if (press) {
    held_key_ = k;
    if (use_sector) {
      onSector(s);
    } else {
      v_ = v_arc; w_ = w_arc;
    }
  } else if (k == held_key_) {
    held_key_ = 0;
    if (use_sector) {
      onSector(TeleopPad::NONE);
    } else {
      v_ = 0.0; w_ = 0.0;
    }
  }
  // Consume so RViz doesn't also act on the same arrow press (some
  // displays accept arrows for navigation).
  return true;
}

void TeleopPanel::onSector(TeleopPad::Sector s) {
  switch (s) {
    case TeleopPad::UP:    v_ =  v_max_; w_ = 0.0; break;
    case TeleopPad::DOWN:  v_ = -v_max_; w_ = 0.0; break;
    case TeleopPad::LEFT:  v_ = 0.0;     w_ =  w_max_; break;
    case TeleopPad::RIGHT: v_ = 0.0;     w_ = -w_max_; break;
    case TeleopPad::STOP:
    case TeleopPad::NONE:
    default:               v_ = 0.0;     w_ = 0.0; break;
  }
}

void TeleopPanel::publishTick() {
  if (!pub_) {
    return;
  }
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = v_;
  msg.angular.z = w_;
  pub_->publish(msg);
}

}  // namespace pro_lab_rviz

PLUGINLIB_EXPORT_CLASS(pro_lab_rviz::TeleopPanel, rviz_common::Panel)
