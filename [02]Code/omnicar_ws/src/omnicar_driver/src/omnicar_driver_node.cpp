#include <rclcpp/rclcpp.hpp>
#include "sdpo_drivers_interfaces/msg/mot_enc.hpp"
#include "sdpo_drivers_interfaces/msg/mot_ref.hpp"
#include <sdpo_serial_port/AsyncSerial.h>
#include <serial_communication_channels/channels.h>

class OmnicarDriver : public rclcpp::Node {
public:
    OmnicarDriver() : Node("omnicar_driver"), channels_('#', '?', ';') {
        // Parametri configurabili
        this->declare_parameter("port", "/dev/omnicar_esp32");
        this->declare_parameter("baud", 115200);
        
        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();

        // Publisher per i dati degli encoder (da ESP a ROS)
        enc_pub_ = this->create_publisher<sdpo_drivers_interfaces::msg::MotEnc>("motors_encoders", 10);

        // Subscriber per i comandi ai motori (da ROS a ESP)
        ref_sub_ = this->create_subscription<sdpo_drivers_interfaces::msg::MotRef>(
            "motors_ref", 10, std::bind(&OmnicarDriver::ref_callback, this, std::placeholders::_1));

        // Inizializzazione Seriale Asincrona (Libreria 5dpo)
        try {
            serial_ = new CallbackAsyncSerial(port, baud);
            serial_->setCallback(std::bind(&OmnicarDriver::on_serial_receive, this, 
                                 std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "OmniCar Driver connesso su %s a %d baud", port.c_str(), baud);
        } catch (boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore Seriale: %s", e.what());
        }
    }

    ~OmnicarDriver() {
        if (serial_) {
            serial_->close();
            delete serial_;
        }
    }

private:
    // Callback: Arrivano dati grezzi dall'ESP32
    void on_serial_receive(const char *data, unsigned int len) {
        std::string incoming(data, len);
        
        // Il parser dei canali gestisce i messaggi tipo #val1;val2;val3;val4?
        if (channels_.Parse(incoming)) {
            auto msg = sdpo_drivers_interfaces::msg::MotEnc();
            // Supponiamo che l'ESP mandi 4 canali per i 4 motori
            for (int i = 0; i < 4; i++) {
                msg.counter.push_back(channels_.GetChannelValue(i));
            }
            enc_pub_->publish(msg);
        }
    }

    // Callback: Arriva un comando di velocità da ROS
    void ref_callback(const sdpo_drivers_interfaces::msg::MotRef::SharedPtr msg) {
        if (!serial_ || !serial_->isOpen()) return;

        // Costruiamo la stringa nel protocollo 5dpo: #vVAL1;VAL2;VAL3;VAL4?
        std::string cmd = "#v";
        for (size_t i = 0; i < msg->slot.size(); i++) {
            cmd += std::to_string(msg->slot[i]);
            if (i < msg->slot.size() - 1) cmd += ";";
        }
        cmd += "?";
        
        serial_->writeString(cmd);
    }

    CallbackAsyncSerial* serial_;
    CChannels channels_;
    rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotEnc>::SharedPtr enc_pub_;
    rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotRef>::SharedPtr ref_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmnicarDriver>());
    rclcpp::shutdown();
    return 0;
}
