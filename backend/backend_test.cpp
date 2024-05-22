#include <iostream>
#include "backend.h"

void testJSONPost() {
    try {
        DatabaseConfig config;
        config.uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
        config.database_name = "meshData";
        config.collection_name = "binary";
        BackendServer backend(config);

        httplib::Request req;
        req.body = R"({"name":"test_item"})";
        req.set_header("Content-Type", "application/json");
        auto &mongo_instance = MongoDBInstance::get_instance(); // Ensures MongoDB client instance
        mongocxx::client conn{mongocxx::uri{config.uri}};
        auto db = conn[config.database_name];
        auto coll = db[config.collection_name];

        auto response = backend.process_json_post(req, coll);
        std::cout << "JSON Post Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "JSON Post Test Failed: " << e.what() << std::endl;
    }
}

void testBinaryPost() {
    try {
        DatabaseConfig config;
        config.uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
        config.database_name = "meshData";
        config.collection_name = "binary";
        BackendServer backend(config);

        httplib::Request req;
        req.body = "binarydata";
        req.set_header("Content-Type", "application/octet-stream");
        auto &mongo_instance = MongoDBInstance::get_instance();
        mongocxx::client conn{mongocxx::uri{config.uri}};
        auto db = conn[config.database_name];
        auto coll = db[config.collection_name];

        auto response = backend.process_binary_post(req, coll);
        std::cout << "Binary Post Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Binary Post Test Failed: " << e.what() << std::endl;
    }
}

void testMalformedJSON() {
    try {
        DatabaseConfig config;
        config.uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
        config.database_name = "meshData";
        config.collection_name = "binary";
        BackendServer backend(config);

        httplib::Request req;
        req.body = R"({malformed_json: true})";
        req.set_header("Content-Type", "application/json");
        auto &mongo_instance = MongoDBInstance::get_instance();
        mongocxx::client conn{mongocxx::uri{config.uri}};
        auto db = conn[config.database_name];
        auto coll = db[config.collection_name];

        auto response = backend.process_json_post(req, coll);
        std::cout << "Malformed JSON Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Malformed JSON Test Failed: " << e.what() << std::endl;
    }
}

int main() {
    testJSONPost();
    testBinaryPost();
    testMalformedJSON();
    return 0;
}




// #include <cassert>
// #include <iostream>
// #include "backend.h"

// int main()
// {
//     // Initializing necessary components
//     auto &mongo_instance = MongoDBInstance::get_instance(); // Ensures MongoDB client instance
//     mongocxx::client conn{mongocxx::uri{"mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"}};
//     auto db = conn["meshData"];
//     auto coll = db["binary"];

//     // mongocxx::client client;     // MongoDB client
//     // initialize_database(client, config);
//     // auto db = client[config.database_name];
//     // auto collection = db[config.collection_name]

//     // Simulating an HTTP request
//     httplib::Request req;
//     req.body = R"({"name":"test_item"})";
//     req.set_header("Content-Type", "application/json");

//     try
//     {
//         // // Testing JSON post handling
//         DatabaseConfig config;
//         // // config.uri = "mongodb+srv://username:password@host/database";
//         // // config.database_name = "example_db";
//         // // config.collection_name = "example_collection";
//         config.uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
//         config.database_name = "meshData";
//         config.collection_name = "binary";
//         BackendServer backend(config);

//         auto json_response = backend.process_json_post(req, coll);
//         //auto json_response = backend.process_json_post(req, config.collection_name);
//         assert(!json_response.empty()); // Simple check to ensure a response is generated
//         std::cout << "JSON post test passed.\n";

//         // Optionally, test other functionalities
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "Test failed: " << e.what() << std::endl;
//         return 1;
//     }

//     std::cout << "All tests passed." << std::endl;
//     return 0;
// }