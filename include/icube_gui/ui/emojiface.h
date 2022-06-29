#ifndef EMOJIFACE_H
#define EMOJIFACE_H

#include <QWidget>
#include <QTimer>
#include <QMouseEvent>
#include <QPropertyAnimation>
#include <QLabel>
#include <QMap>

namespace Ui {
class EmojiFace;
}

class EmojiFace : public QWidget
{
    Q_OBJECT

public:
    enum class Mood{
      HAPPY,
      SAD,
      NERVOUS,
      ANGRY,
      SLEEPY,
      DEAD
    };

    explicit EmojiFace(QWidget *parent = nullptr);
    ~EmojiFace();

    void setMood(EmojiFace::Mood mood);

protected:
    void showEvent (QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

signals:
    /**
     * @brief This signal will be emitted when a mouse press event occurs in Emoji widget area
     */
    void pressed(void);

    /**
     * @brief moodChange
     * @param mood
     */
    void moodChange(EmojiFace::Mood mood);

private slots:
    /**
     * @brief changeMood : This slot is used to decouple setMood callbacks from external threads
     * @param mood
     */
    void changeMood(EmojiFace::Mood mood);
private:
    enum class EmojiComponent{
        FACE,
        EYES,
        MOUTH
    };

    Ui::EmojiFace *ui;
    QMap<EmojiFace::EmojiComponent , QLabel*> emoji_widget_;

    int eye_widget_default_x_ = 50; // Widget Middle Position
    int eye_widget_max_movement_distance_ = 60;

    void setImage(EmojiFace::EmojiComponent component, QString image_url);
    void animateEyes(void);  

};

#endif // EMOJIFACE_H
