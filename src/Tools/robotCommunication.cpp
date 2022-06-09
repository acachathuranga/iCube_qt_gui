#include "icube_gui/Tools/robotCommunication.h"

RobotCommunication::RobotCommunication(Console *console)
{
    console_ = console;
    cli = new mqtt::async_client(SERVER_ADDRESS, CLIENT_ID);
    keep_alive_ = true;

    // Connect MQTT Client
    connect_client();

    /// Initialize Connection status checker
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &RobotCommunication::check_status);
    timer_->start(1000);
}

RobotCommunication::~RobotCommunication()
{
    keep_alive_ = false;
    end_communication();
    delete cli;
}

void RobotCommunication::publish(std::string topic, std::string msg)
{
    auto payload = mqtt::make_message(topic, msg);
    payload->set_qos(QOS);
    try{
        cli->publish(payload);
    }
    catch(const mqtt::exception& exc){
        console_->print("Error: Mqtt exception. Attempting reconnection!");
        connect_client();
    }

}

void RobotCommunication::on_message(mqtt::const_message_ptr msg)
{
    std::unique_lock<std::mutex> lck (subscriber_map_mutex_);
    std::string topic = msg->get_topic();
    if (callbacks_.contains(topic))
    {
        Q_FOREACH (const boost::function<void (std::string)> &cb, callbacks_[topic])
        {
            if (cb != nullptr)
            {
                cb(msg->get_payload_str());
            }
            else
            {
                console_->print("Error: Nullpointer callback encountered for registered topic [" + topic + "]");
            }

        }
    }
}

/**
 * @brief RobotCommunication::subscribe
 * @param topic
 * @param callback : Message reception and callbacks will be handled in the same thread.
 *                   Hence callback functions should not be blocking for extended periods of time
 */
void RobotCommunication::subscribe(std::string topic, boost::function<void (std::string)> callback)
{
    std::unique_lock<std::mutex> lck (subscriber_map_mutex_);
    callbacks_[topic].append(callback);
}

void RobotCommunication::publish(std::string topic, std::string field, bool value)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    publish(topic, writer.write(message_json));
}

void RobotCommunication::publish(std::string topic, std::string field, bool value, std::string optional_msg)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    message_json["message"] = optional_msg;
    publish(topic, writer.write(message_json));
}

void RobotCommunication::publish(std::string topic, std::string field, std::string value)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    publish(topic, writer.write(message_json));
}

void RobotCommunication::end_communication()
{
    keep_alive_ = false;
    try {
        // Disconnect MQTT

        // Shutting down and disconnecting from the MQTT server
        cli->unsubscribe("#");
        cli->stop_consuming();
        cli->disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::connect_client()
{
    connOpts.set_keep_alive_interval(36000);
    connOpts.set_clean_session(true);

    try {
        //cli->set_connected_handler([this](const std::string& cause){connected_cb(cause);});
        cli->connect(connOpts)->wait();
        cli->set_message_callback([this](mqtt::const_message_ptr msg) {on_message(msg);});
        console_->print("Connected to Mqtt Server: " + SERVER_ADDRESS + ". Client: " + CLIENT_ID);
        cli->subscribe("#", QOS)->wait();
        console_->print("Subscribed to all topics");

    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::reconnect_client()
{
    try {
        cli->reconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::connected_cb(const std::string& cause)
{
    Q_UNUSED(cause);
    //console_->print("Connected to Mqtt Server: " + SERVER_ADDRESS + ". Client: " + CLIENT_ID);
    //cli->subscribe("#", QOS);
    //console_->print("Subscribed to all topics");
}

void RobotCommunication::disconnected_cb(const mqtt::properties &properties, mqtt::ReasonCode cause)
{
    Q_UNUSED(properties);
    Q_UNUSED(cause);

    console_->print("Warning: Mqtt disconnected. Reconnecting now...");
    if (keep_alive_)
    {
        cli->connect(connOpts);
    }
}

void RobotCommunication::check_status(void)
{
    if (!cli->is_connected())
    {
        console_->print("Error: WiFi Network Disconnected. Attempting MQTT reconnection!");
        connect_client();

    }
}
