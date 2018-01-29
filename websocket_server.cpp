#include <functional>
#include <set>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "json.hpp"


using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;

using json = nlohmann::json;

using namespace std;

namespace wsServer {

class touch_server {
    typedef websocketpp::server<websocketpp::config::asio> server;

public:
    touch_server() {
        m_server.init_asio();

        m_server.set_open_handler(bind(&touch_server::on_open,this,_1));
        m_server.set_close_handler(bind(&touch_server::on_close,this,_1));
        m_server.set_reuse_addr(true);

        m_server.set_access_channels(websocketpp::log::alevel::none);
    }

    void on_open(connection_hdl hdl) {
        m_connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        m_connections.erase(hdl);
    }

    void sendJSON(const json& j) {
        for (auto it : m_connections) {
            m_server.send(it, j.dump(), websocketpp::frame::opcode::text);
        }
    }

    void stop() {
        cout << "stop listening" << endl;
        m_server.stop_listening();
        cout << "close connections" << endl;
        for(connection_hdl c : m_connections) {
            m_server.get_con_from_hdl(c)->close(0, "shutdown");
        }
        cout << "stop server" << endl;
        m_server.stop();
    }

    void run() {
        m_server.listen(9002);
        m_server.start_accept();
        m_server.run();
    }
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server m_server;
    con_list m_connections;
};

touch_server server;
std::thread network_thread;
} /* wsServer */


int startWSServer() {
    wsServer::network_thread = std::thread(&wsServer::touch_server::run, &wsServer::server);
    cout << "WebSocket server started." << endl;
}

void stopWSServer() {
    wsServer::server.stop();
    wsServer::network_thread.join();
    cout << "WebSocket server stopped." << endl;
}

void serverSend(const json& j) {
    wsServer::server.sendJSON(j);
}
