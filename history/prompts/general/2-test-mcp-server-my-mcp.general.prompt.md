---
id: 2
title: Test MCP Server `my-mcp`
stage: general
date_iso: 2025-12-06
surface: agent
model: claude-3-5-sonnet-20240620
feature: none
branch: main
user: user
command: mcp test my-mcp
labels: ["mcp", "test"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - mcp-config.json
  - mcp_server.py
tests_yaml: []
prompt_text: |
  mcp test my-mcp
response_text: |
  llmContent                                 returnDisplay
  ----------                                 -------------
  Received input: {'test_key': 'test_value'} MCP Server is working!
outcome: success
evaluation: The MCP server 'my-mcp' was successfully tested, indicating it is running and responding as expected.
---
