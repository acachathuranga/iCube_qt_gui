#ifndef ROBOTCOMMUNICATION_H
#define ROBOTCOMMUNICATION_H

#include <QObject>
#include <QTimer>
#include <QList>
#include <QMap>

#include <mqtt/async_client.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <mutex>

#include <icube_gui/Tools/console.h>
#include <jsoncpp/json/json.h>


class RobotCommunication : public QObject
{
    Q_OBJECT
    public:
        RobotCommunication(Console *console);
        ~RobotCommunication();

        void end_communication();
        void publish(std::string topic, std::string msg);
        void publish(std::string topic, std::string field, bool value);
        void publish(std::string topic, std::string field, bool value, std::string optional_msg);
        void publish(std::string topic, std::string field, std::string value);
        void subscribe(std::string topic, boost::function<void (std::string)>);

    private:
        //const std::string SERVER_ADDRESS	{ "tcp://192.168.5.128:1883" };
        const std::string SERVER_ADDRESS	{ "localhost:1883" };
        const std::string CLIENT_ID         { "robot_ui" };
        const int  QOS = 1;
        bool keep_alive_ = true;

        mqtt::connect_options connOpts;
        mqtt::async_client* cli;

        QMap<std::string, QList<boost::function<void (std::string)>>> callbacks_;
        Console *console_;
        QTimer *timer_;

        std::mutex subscriber_map_mutex_;

        void connected_cb(const std::string& cause);
        void disconnected_cb(const mqtt::properties &properties, mqtt::ReasonCode cause);
        void connect_client();
        void reconnect_client();
        void check_status(void);
        void on_message(mqtt::const_message_ptr msg);
};

#endif // ROBOTCOMMUNICATION_H
