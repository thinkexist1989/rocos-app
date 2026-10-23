#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "src/app_config.hpp"
#include "test/doctest.h"

#include <cstdlib>
#include <fstream>

namespace {
namespace fs = std::filesystem;

struct ConfigFixture {
    fs::path directory;
    fs::path original_directory = fs::current_path();

    ConfigFixture() {
        char pattern[] = "/tmp/rocos-app-config-XXXXXX";
        const char* path = mkdtemp(pattern);
        REQUIRE(path != nullptr);
        directory = path;
    }

    ~ConfigFixture() {
        fs::current_path(original_directory);
        fs::remove_all(directory);
    }

    rocos::AppConfig load(const std::string& text) {
        const auto file = directory / "config.yaml";
        std::ofstream(file) << text;
        return rocos::AppConfig::load(file.string());
    }
};
}  // namespace

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig loads startup parameters and resolves paths beside YAML") {
    fs::create_directory(directory / "unrelated-cwd");
    fs::current_path(directory / "unrelated-cwd");
    const auto config = load(R"(
http_host: 127.0.0.1
http_port: 12345
http_control_ttl_seconds: 30
robot:
  hardware_config: hardware/driver.yaml
  ecat_id: 2
  urdf: models/robot.urdf
  base_link: base
  tip_link: tool
  joint_binding: bindings.yaml
  payload: load.yaml
  frames: tools/calibration.yaml
  id: robot_A
  tool_library: ../tool-library
scripts: lua
web_root: ../web
)");
    CHECK(config.http_host == "127.0.0.1");
    CHECK(config.http_port == 12345);
    CHECK(config.http_control_ttl_seconds == 30);
    CHECK(config.robot.ecat_id == 2);
    CHECK(config.robot.base_link == "base");
    CHECK(config.robot.tip_link == "tool");
    CHECK(config.robot.hardware_config == (directory / "hardware/driver.yaml").string());
    CHECK(config.robot.urdf == (directory / "models/robot.urdf").string());
    CHECK(config.robot.joint_binding == (directory / "bindings.yaml").string());
    CHECK(config.robot.payload == (directory / "load.yaml").string());
    CHECK(config.robot.frames == (directory / "tools/calibration.yaml").string());
    CHECK(config.robot.id == "robot_A");
    CHECK(config.robot.tool_library == (directory.parent_path() / "tool-library").string());
    CHECK(config.scripts == (directory / "lua").string());
    CHECK(config.web_root == (directory.parent_path() / "web").string());
    CHECK(fs::current_path() == directory / "unrelated-cwd");
}

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig supports absolute resource paths and relative config paths") {
    fs::current_path(directory);
    const auto config = load("robot:\n  urdf: /opt/robot/model.urdf\nscripts: /opt/robot/scripts\n");
    CHECK(config.robot.urdf == "/opt/robot/model.urdf");
    CHECK(config.scripts == "/opt/robot/scripts");
    const auto relative = rocos::AppConfig::load("config.yaml");
    CHECK(relative.robot.hardware_config == (directory / "hardware_talon_config.yaml").string());
    CHECK(relative.robot.urdf == config.robot.urdf);
    CHECK(relative.http_port == 8080);
    CHECK(relative.http_control_ttl_seconds == 300);
}

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig rejects missing malformed and invalid settings") {
    CHECK_THROWS(rocos::AppConfig::load((directory / "missing.yaml").string()));
    for (const auto* yaml :
         {"", "[]", "http_port: [", "http_port: invalid", "http_port: 0", "http_port: 65536", "http_host: ''",
          "http_control_ttl_seconds: 0", "http_control_ttl_seconds: -1", "robot: []", "robot:\n  ecat_id: -1",
          "robot:\n  urdf: ''", "robot:\n  base_link: ''", "scripts: ''"}) {
        CAPTURE(yaml);
        CHECK_THROWS(load(yaml));
    }
}

TEST_CASE("Repository startup config points to existing source resources") {
    const auto config = rocos::AppConfig::load(ROCOS_TEST_CONFIG_FILE);
    CHECK(std::filesystem::is_regular_file(config.robot.hardware_config));
    CHECK(std::filesystem::is_regular_file(config.robot.urdf));
    CHECK(std::filesystem::is_regular_file(config.robot.joint_binding));
    CHECK(std::filesystem::is_regular_file(config.robot.payload));
    CHECK(std::filesystem::is_directory(config.scripts));
    CHECK(std::filesystem::is_directory(config.web_root));
}

TEST_CASE_FIXTURE(ConfigFixture, "Logging and diagnostics resolve paths and validate independent limits") {
    auto config = load(R"(
logging:
  directory: diagnostic-logs
  file_level: trace
  console_level: critical
  max_file_size_mib: 2
  max_files: 3
  flush_interval_ms: 20
diagnostics:
  host: 127.0.0.1
  port: 8099
  max_sse_clients: 2
  max_downloads: 1
)");
    CHECK(config.logging.directory == (directory / "diagnostic-logs").string());
    CHECK(config.logging.file_level == rocos::LogLevel::Trace);
    CHECK(config.logging.console_level == rocos::LogLevel::Critical);
    CHECK(config.logging.max_file_size == 2 * 1024 * 1024);
    CHECK(config.logging.max_files == 3);
    CHECK(config.diagnostics.port == 8099);
    CHECK(config.diagnostics.max_sse_clients == 2);
    for (auto yaml : {"logging: []", "logging: {directory: ''}", "logging: {file_level: error}",
                      "logging: {console_level: unknown}", "logging: {max_file_size_mib: -1}",
                      "logging: {max_files: 0}", "logging: {flush_interval_ms: 0}", "diagnostics: []",
                      "diagnostics: {port: 8080}", "diagnostics: {port: 0}", "diagnostics: {max_sse_clients: 0}",
                      "diagnostics: {max_downloads: 17}"}) {
        CAPTURE(yaml);
        CHECK_THROWS(load(yaml));
    }
}
