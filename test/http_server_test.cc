#include <rocos_app/robot_http_server.hpp>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

// Tests only control rights, validation and unsupported routes. No Robot or
// hardware instance is created, and no movement or enable command is issued.
int main() {
    rocos::RobotHttpServer server(nullptr);
    const int port = 18973;
    server.runAsync("127.0.0.1", port);
    httplib::Client client("127.0.0.1", port);
    client.set_connection_timeout(1);
    client.set_read_timeout(2);
    client.set_keep_alive(true);

    auto check = [](bool ok, const char* message) {
        if (!ok) throw std::runtime_error(message);
    };
    auto json = [&](const httplib::Result& response) {
        check(static_cast<bool>(response), "HTTP request failed");
        check(response->status == 200, "Unexpected HTTP status");
        return nlohmann::json::parse(response->body);
    };

    try {
        bool ready = false;
        for (int i = 0; i < 100; ++i) {
            auto response = client.Get("/api/control/status");
            if (response) { ready = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(ready, "HTTP server did not start");
        {
            rocos::RobotHttpServer duplicate(nullptr);
            bool rejected = false;
            try {
                duplicate.runAsync("127.0.0.1", port);
            } catch (const std::runtime_error&) {
                rejected = true;
            }
            check(rejected, "Occupied HTTP port did not report startup failure");
        }
        auto status = json(client.Get("/api/control/status"));
        check(status["success"] == true, "Control status failed");
        check(status["data"]["has_owner"] == false, "Unexpected control owner");

        auto denied = json(client.Post("/api/script/run", "{}", "application/json"));
        check(denied["success"] == false, "Write without control token accepted");
        check(denied["error_name"] == "ControlTokenMissing", "Missing control error name");
        auto acquired = json(client.Post("/api/control/acquire",
            R"({"client_name":"http-test","client_id":"http-test"})", "application/json"));
        check(acquired["success"] == true, "Control acquisition failed");
        httplib::Headers headers = {
            {"X-Rocos-Control-Token", acquired["data"]["token"].get<std::string>()},
            {"X-Rocos-Client-Id", acquired["data"]["client_id"].get<std::string>()}
        };
        for (const auto* path : {"/api/script/run", "/api/robot/pause",
                                "/api/robot/servo/start"}) {
            auto response = json(client.Post(path, headers, "{}", "application/json"));
            check(response["success"] == false && response["code"] == 3007,
                  "Unsupported write did not report code 3007");
            check(response["error_name"] == "FunctionNotSupported", "Unsupported error name is ambiguous");
        }
        auto unsupported = json(client.Get("/api/script/status"));
        check(unsupported["success"] == false && unsupported["code"] == 3007,
              "Unsupported read did not report code 3007");
        auto invalid = json(client.Post("/api/robot/workmode", headers, "{", "application/json"));
        check(invalid["success"] == false && invalid["code"] == 1001,
              "Malformed JSON was not rejected");
        auto jerk = json(client.Post("/api/robot/movej_ik", headers,
            R"({"pose":{},"jerk":0})", "application/json"));
        check(jerk["success"] == false && jerk["code"] == 1001,
              "Invalid jerk parameter was not rejected before motion");
        auto bad_type = json(client.Post("/api/robot/movel", headers,
            R"({"pose":{},"velocity":"fast"})", "application/json"));
        check(bad_type["success"] == false && bad_type["code"] == 1001,
              "JSON type exception was not converted to API error");
        auto released = json(client.Post("/api/control/release", headers, "{}", "application/json"));
        check(released["success"] == true, "Control release failed");
        client.stop();
        server.stop();
        // runAsync must return only after listening so immediate stop is safe.
        server.runAsync("127.0.0.1", port);
        server.stop();
        std::cout << "HTTP control, validation and unsupported-route checks passed\n";
    } catch (const std::exception& e) {
        client.stop();
        server.stop();
        std::cerr << e.what() << '\n';
        return 1;
    }
}
