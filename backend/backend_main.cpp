#include "backend.h"

// Initialize MongoDB client using the singleton instance
void initialize_database(mongocxx::client& client, const DatabaseConfig& config) {
    auto& instance = MongoDBInstance::get_instance();  // Ensures single instance
    client = mongocxx::client{mongocxx::uri{config.uri}};
}

// Handle JSON post request
std::string process_json_post(const httplib::Request& req, mongocxx::collection& collection) {
    try {
        auto doc_value = bsoncxx::from_json(req.body);
        auto result = collection.insert_one(doc_value.view());
        auto response_json = bsoncxx::builder::basic::make_document(
            bsoncxx::builder::basic::kvp("message", "Data received successfully"),
            bsoncxx::builder::basic::kvp("id", result->inserted_id().get_oid().value.to_string())
        );
        return bsoncxx::to_json(response_json);
    } catch (const std::exception& e) {
        return R"({"error": "Error processing your request."})";
    }
}

// Handle binary post request
std::string process_binary_post(const httplib::Request& req, mongocxx::collection& collection) {
    try {
        bsoncxx::types::b_binary binary_data{
            bsoncxx::binary_sub_type::k_binary,
            static_cast<uint32_t>(req.body.size()),
            reinterpret_cast<const uint8_t*>(req.body.data())
        };

        auto document = bsoncxx::builder::stream::document{};
        bsoncxx::document::value doc_value = document
            << "name" << "UnnamedMesh"
            << "data" << binary_data
            << "timestamp" << bsoncxx::types::b_date{std::chrono::system_clock::now()}
            << bsoncxx::builder::stream::finalize;

        auto result = collection.insert_one(doc_value.view());
        auto response_json = bsoncxx::builder::basic::make_document(
            bsoncxx::builder::basic::kvp("message", "Binary data received successfully"),
            bsoncxx::builder::basic::kvp("id", result->inserted_id().get_oid().value.to_string())
        );
        return bsoncxx::to_json(response_json);
    } catch (const std::exception& e) {
        return R"({"error": "Error saving binary data."})";
    }
}

int main() {
    DatabaseConfig config;
    mongocxx::client client;     // MongoDB client
    initialize_database(client, config);
    auto db = client[config.database_name];
    auto collection = db[config.collection_name];

    httplib::Server svr;

    svr.Post("/upload", [&](const httplib::Request& req, httplib::Response& res) {
        std::string contentType = req.get_header_value("Content-Type");

        if (contentType == "application/json") {
            std::string response = process_json_post(req, collection);
            res.set_content(response, "application/json");
        } else if (contentType == "application/octet-stream") {
            std::string response = process_binary_post(req, collection);
            res.set_content(response, "application/json");
        } else {
            res.status = 400; // Bad Request
            res.set_content(R"({"error": "Unsupported content type."})", "application/json");
        }
    });

    std::cout << "Starting server on port 8080..." << std::endl;
    if (!svr.listen("0.0.0.0", 8080)) {
        std::cerr << "Failed to start the server. Make sure the port is not in use." << std::endl;
        return 1;
    }

    return 0;
}
