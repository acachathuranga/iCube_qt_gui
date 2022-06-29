#include "icube_gui/ui/emojiface.h"
#include "ui_emojiface.h"

EmojiFace::EmojiFace(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EmojiFace)
{
    ui->setupUi(this);

    emoji_widget_[EmojiComponent::FACE] = ui->face_label;
    emoji_widget_[EmojiComponent::EYES] = ui->eyes_label;
    emoji_widget_[EmojiComponent::MOUTH] = ui->mouth_label;

    setMood(Mood::HAPPY);

    /// Eyes Animation Timer
    QTimer *animation_timer = new QTimer(this);
    connect(animation_timer, &QTimer::timeout, this, &EmojiFace::animateEyes);
    animation_timer->start(3000);

    qRegisterMetaType<EmojiFace::Mood>("EmojiFace::Mood");
    connect(this, &EmojiFace::moodChange, this, &EmojiFace::changeMood);
}

EmojiFace::~EmojiFace()
{
    delete ui;
}

void EmojiFace::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
}

void EmojiFace::hideEvent(QHideEvent *event)
{
    QWidget::hideEvent(event);
}

void EmojiFace::mousePressEvent(QMouseEvent *event)
{
    emit pressed();
    QWidget::mousePressEvent(event);
}

void EmojiFace::setMood(EmojiFace::Mood mood)
{
    emit moodChange(mood);
}

void EmojiFace::changeMood(EmojiFace::Mood mood)
{
    switch (mood)
    {
        case EmojiFace::Mood::HAPPY:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/happy_face.png");
            setImage(EmojiComponent::EYES, ":/images/Emoji/images/Emoji/eyes.png");
            setImage(EmojiComponent::MOUTH, ":/images/Emoji/images/Emoji/happy_mouth.png");
            break;
        case EmojiFace::Mood::SAD:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/sad_face.png");
            setImage(EmojiComponent::EYES, ":/images/Emoji/images/Emoji/sad_eyes.png");
            setImage(EmojiComponent::MOUTH, ":/images/Emoji/images/Emoji/sad_mouth.png");
            break;
        case EmojiFace::Mood::NERVOUS:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/happy_face.png");
            setImage(EmojiComponent::EYES, ":/images/Emoji/images/Emoji/eyes.png");
            setImage(EmojiComponent::MOUTH, ":/images/Emoji/images/Emoji/nervous_mouth.png");
            break;
        case EmojiFace::Mood::ANGRY:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/scream_face.png");
            setImage(EmojiComponent::EYES, ":/images/Emoji/images/Emoji/eyes.png");
            setImage(EmojiComponent::MOUTH, ":/images/Emoji/images/Emoji/scream_mouth.png");
            break;
        case EmojiFace::Mood::SLEEPY:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/sleepy_face.png");
            setImage(EmojiComponent::EYES, ":/images/Emoji/images/Emoji/sleepy_eyes.png");
            setImage(EmojiComponent::MOUTH, "");
            break;
        case EmojiFace::Mood::DEAD:
            setImage(EmojiComponent::FACE, ":/images/Emoji/images/Emoji/dead_face.png");
            setImage(EmojiComponent::EYES, "");
            setImage(EmojiComponent::MOUTH, "");
            break;
    }
}

void EmojiFace::setImage(EmojiFace::EmojiComponent component, QString image_url)
{
    QPixmap pix_map_image;
    if (QFile::exists(image_url))
    {
        pix_map_image = QPixmap(image_url);
        pix_map_image = pix_map_image.scaledToWidth( 1800 );
    }
    emoji_widget_[component]->setPixmap(pix_map_image);
}

void EmojiFace::animateEyes(void)
{
    int x = emoji_widget_[EmojiComponent::EYES]->x();
    int y = emoji_widget_[EmojiComponent::EYES]->y();
    int width = emoji_widget_[EmojiComponent::EYES]->width();
    int height = emoji_widget_[EmojiComponent::EYES]->height();

    /// Calculate random distance within + or - max_movement_distance
    int motion_distance = (qrand() % eye_widget_max_movement_distance_) * 2 - eye_widget_max_movement_distance_;

    QPropertyAnimation *animation = new QPropertyAnimation(emoji_widget_[EmojiComponent::EYES], "geometry");
    animation->setDuration(1000);
    animation->setStartValue(QRect(x, y, width, height));
    animation->setEndValue(QRect(eye_widget_default_x_+motion_distance, y, width, height));
    animation->start();
}
