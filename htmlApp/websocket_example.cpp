#include <functional>
#include <set>
#include <csignal>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "../json.hpp"


using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;

using json = nlohmann::json;

using namespace std;


class count_server {
    typedef websocketpp::server<websocketpp::config::asio> server;

public:
    count_server() : m_count(0) {
        m_server.init_asio();

        m_server.set_open_handler(bind(&count_server::on_open,this,_1));
        m_server.set_close_handler(bind(&count_server::on_close,this,_1));

        m_timer = m_server.set_timer(1000, bind(&count_server::count,this,_1));
    }

    void on_open(connection_hdl hdl) {
        m_connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        m_connections.erase(hdl);
    }

    void count(const websocketpp::lib::error_code & ec) {
        m_count++;

        std::stringstream ss;
        json j = {
                    {{"x", 100}, {"y", 150}},
                    {{"x", 130}, {"y", 250}},
                    {{"x", 150}, {"y", 50}}
                 };

        for (auto it : m_connections) {
            m_server.send(it, j.dump(), websocketpp::frame::opcode::text);
        }

        m_timer = m_server.set_timer(1000, bind(&count_server::count,this,_1));
    }

    void stop() {
        m_server.stop();
    }

    void run(uint16_t port) {
        m_server.listen(port);
        m_server.start_accept();
        m_server.run();
    }
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    int m_count;
    server m_server;
    con_list m_connections;
    server::timer_ptr m_timer;
};

count_server server;

void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received.\n";
    server.stop();

}

int main() {
    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    server.run(9002);

    cout << "shutdown." << endl;
}
