from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route("/mcp", methods=["POST"])
def mcp():
    data = request.json
    return jsonify({
        "llmContent": f"Received input: {data}",
        "returnDisplay": "MCP Server is working!"
    })

if __name__ == "__main__":
    app.run(port=8000)
