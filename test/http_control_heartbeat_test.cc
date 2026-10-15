#include <rocos_app/robot_http_server.hpp>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

// Real HTTP and steady-clock expiry, with no Robot or hardware instance.
int main() {
    using namespace std::chrono;
    rocos::RobotHttpServer server(nullptr, nullptr, 1);
    const int port = 18974;
    httplib::Client client("127.0.0.1", port);
    client.set_connection_timeout(1);
    client.set_read_timeout(2);
    auto check = [](bool value, const char* message) {
        if (!value) throw std::runtime_error(message);
    };
    auto json = [&](const httplib::Result& response) {
        check(response && response->status == 200, "HTTP request failed");
        return nlohmann::json::parse(response->body);
    };
    auto post = [&](const char* path, const httplib::Headers& headers = httplib::Headers{}) {
        return json(client.Post(path, headers, "{}", "application/json"));
    };
    auto credentials = [](const nlohmann::json& acquired) {
        return httplib::Headers{
            {"X-Rocos-Control-Token", acquired["data"]["token"].get<std::string>()},
            {"X-Rocos-Client-Id", acquired["data"]["client_id"].get<std::string>()}};
    };
    auto acquire = [&]() {
        auto result = json(client.Post("/api/control/acquire",
            R"({"client_id":"heartbeat-owner","client_name":"heartbeat-test"})", "application/json"));
        check(result["success"] == true, "Acquire failed");
        return result;
    };
    try {
        bool invalid_config = false;
        try { rocos::RobotHttpServer invalid(nullptr, nullptr, 0); }
        catch (const std::invalid_argument&) { invalid_config = true; }
        check(invalid_config, "Nonpositive TTL accepted");
        server.runAsync("127.0.0.1", port);
        auto acquired = acquire();
        auto owner = credentials(acquired);
        check(acquired["data"]["ttl_seconds"] == 1, "TTL configuration ignored");
        check(post("/api/control/heartbeat")["code"] == 3006, "Missing token accepted");
        auto missing_id = owner;
        missing_id.erase("X-Rocos-Client-Id");
        check(post("/api/control/heartbeat", missing_id)["code"] == 3007, "Missing client ID accepted");
        auto wrong = owner;
        wrong.erase("X-Rocos-Client-Id");
        wrong.emplace("X-Rocos-Client-Id", "another-client");
        check(post("/api/control/heartbeat", wrong)["code"] == 3007, "Wrong client ID accepted");
        check(post("/api/control/heartbeat", {{"X-Rocos-Control-Token", "debug-token"},
            {"X-Rocos-Client-Id", "debug-client"}})["code"] == 3007, "Debug token renewed a lease");

        const auto started = steady_clock::now();
        do {
            auto beat = post("/api/control/heartbeat", owner);
            check(beat["success"] == true && beat["code"] == 0, "Owner heartbeat rejected");
            check(!beat["data"].contains("token"), "Heartbeat leaked token");
            check(beat["data"]["owner_client_id"] == "heartbeat-owner", "Wrong heartbeat owner");
            check(post("/api/control/acquire")["code"] == 3008, "Live lease lost exclusivity");
            std::this_thread::sleep_for(milliseconds(100));
        } while (steady_clock::now() - started < milliseconds(1500));
        // New TCP connection, same credentials: identity does not depend on source port.
        httplib::Client second_connection("127.0.0.1", port);
        check(json(second_connection.Post("/api/control/heartbeat", owner, "{}", "application/json"))
            ["success"] == true, "Changing TCP connections invalidated the session");
        second_connection.stop();
        // Neither owner GETs, observer GETs, nor mismatched heartbeats renew the lease.
        const auto idle = steady_clock::now();
        while (steady_clock::now() - idle < milliseconds(1200)) {
            json(client.Get("/api/control/status", owner));
            json(client.Get("/api/control/status"));
            check(post("/api/control/heartbeat", wrong)["code"] == 3007, "Wrong heartbeat accepted");
            std::this_thread::sleep_for(milliseconds(100));
        }
        check(post("/api/control/heartbeat", owner)["code"] == 3007, "Expired heartbeat revived lease");
        check(json(client.Get("/api/control/status"))["data"]["has_owner"] == false, "GET renewed lease");

        auto reacquired = acquire();
        check(reacquired["data"]["token"] != acquired["data"]["token"], "Reused expired token");
        owner = credentials(reacquired);
        auto taken = json(client.Post("/api/control/takeover", R"({"client_id":"new-owner"})", "application/json"));
        check(taken["success"] == true, "Explicit takeover failed");
        check(post("/api/control/heartbeat", owner)["code"] == 3007, "Takeover left old token valid");
        check(post("/api/control/release", owner)["code"] == 3007, "Old owner released new lease");
        auto new_owner = credentials(taken);
        check(post("/api/control/heartbeat", new_owner)["success"] == true, "New owner cannot renew");
        check(post("/api/control/release", new_owner)["success"] == true, "Release failed");
        check(post("/api/control/heartbeat", new_owner)["code"] == 3007, "Released token revived lease");
        auto final_owner = credentials(acquire());
        std::this_thread::sleep_for(milliseconds(1100));
        check(post("/api/control/release", final_owner)["code"] == 3007, "Release accepted expired token");
        server.stop();
        std::cout << "Heartbeat renewal, expiry, identity, takeover and release passed\n";
        return 0;
    } catch (const std::exception& error) {
        client.stop();
        server.stop();
        std::cerr << error.what() << '\n';
        return 1;
    }
}
