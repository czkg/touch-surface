#ifndef _WEBSOCKET_SERVER_H
#define _WEBSOCKET_SERVER_H


#include "json.hpp"

int startWSServer();
void serverSend(const nlohmann::json& j);
void stopWSServer();


#endif /* end of include guard: _WEBSOCKET_SERVER_H */
