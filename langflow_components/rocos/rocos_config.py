"""
ROCOS 全局配置组件 — 设置 ROCOS HTTP API 的基础地址。

该组件不需要连接到 Agent，只需在 Flow 中放置一次即可。
其他所有 Rocos 组件通过读取该组件的输出来获取 base_url。
"""

import os
from langflow.custom import Component
from langflow.io import Output, StrInput
from langflow.schema import Data


class RocosConfig(Component):
    """ROCOS 机器人配置组件 — 基础 URL 和应用设置"""

    display_name: str = "Rocos 配置"
    description: str = (
        "配置 ROCOS 机器人 HTTP API 的连接参数。"
        "设置 base_url（如 http://localhost:8080）后，其他 Rocos 组件可共享此配置。"
    )
    icon: str = "Settings"
    name: str = "rocos_config"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS HTTP API 的基础 URL，例如 http://localhost:8080 或 http://192.168.1.100:8080",
            value=os.environ.get("ROCOS_BASE_URL", "http://localhost:8080"),
            required=True,
        ),
    ]

    outputs = [
        Output(
            display_name="Config",
            name="config",
            method="build_config_data",
        ),
    ]

    def build_config_data(self) -> Data:
        base_url = self.base_url.rstrip("/") if self.base_url else "http://localhost:8080"
        self.status = f"ROCOS API: {base_url}"
        return Data(
            text=f"ROCOS API 已配置于 {base_url}",
            data={
                "base_url": base_url,
            },
        )
