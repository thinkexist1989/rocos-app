"""
Shared HTTP helpers for ROCOS Langflow components.
"""

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional


def _error(code: int, message: str) -> dict:
    return {"success": False, "code": code, "message": message, "data": None}


def _url(base_url: str, path: str, params: Optional[dict] = None) -> str:
    url = f"{base_url.rstrip('/')}{path}"
    if params:
        query = urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
        if query:
            url = f"{url}?{query}"
    return url


def _decode_json(body: str) -> dict:
    if not body:
        return {}
    return json.loads(body)


def _parse_http_error(e: urllib.error.HTTPError) -> dict:
    try:
        body = e.read().decode("utf-8")
        parsed = _decode_json(body)
        if isinstance(parsed, dict):
            parsed.setdefault("success", False)
            parsed.setdefault("code", e.code)
            parsed.setdefault("message", e.reason)
            return parsed
    except Exception:
        pass
    return _error(e.code, f"HTTP {e.code}: {e.reason}")


def _rocos_request(
    base_url: str,
    path: str,
    method: str = "GET",
    body: Optional[dict] = None,
    params: Optional[dict] = None,
    expect_json: bool = True,
    timeout: float = 30.0,
):
    url = _url(base_url, path, params)
    data = json.dumps(body).encode("utf-8") if body is not None else None
    headers = {"Accept": "application/json" if expect_json else "*/*"}
    if data is not None:
        headers["Content-Type"] = "application/json"

    try:
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8")
            return _decode_json(text) if expect_json else text
    except urllib.error.HTTPError as e:
        return _parse_http_error(e)
    except urllib.error.URLError as e:
        return _error(-1, f"网络错误: {e}")
    except json.JSONDecodeError as e:
        return _error(-2, f"JSON 解析错误: {e}")


def _rocos_get(base_url: str, path: str, params: Optional[dict] = None) -> dict:
    return _rocos_request(base_url, path, method="GET", params=params, timeout=10.0)


def _rocos_post(base_url: str, path: str, body: Optional[dict] = None) -> dict:
    return _rocos_request(base_url, path, method="POST", body=body or {})


def _rocos_delete(base_url: str, path: str, params: Optional[dict] = None) -> dict:
    return _rocos_request(base_url, path, method="DELETE", params=params)


def _rocos_text(base_url: str, path: str, params: Optional[dict] = None) -> str:
    return _rocos_request(
        base_url, path, method="GET", params=params, expect_json=False, timeout=10.0
    )


def parse_float_list(text: str, expected_len: Optional[int] = None) -> list[float]:
    values = [float(x.strip()) for x in text.split(",") if x.strip()]
    if expected_len is not None and len(values) != expected_len:
        raise ValueError(f"expected {expected_len} values, got {len(values)}")
    return values
