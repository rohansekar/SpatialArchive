#ifndef BACKEND_H
#define BACKEND_H

#include "httplib.h"
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <string>
#include <chrono>

// MongoDB access details
struct DatabaseConfig {
    std::string uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
    std::string database_name = "meshData";
    std::string collection_name = "binary";
};

// Singleton for MongoDB instance management
class MongoDBInstance {
public:
    static mongocxx::instance& get_instance() {
        static mongocxx::instance instance;
        return instance;
    }

private:
    MongoDBInstance() {}
};

class BackendServer {
public:
    BackendServer(const DatabaseConfig& config);
    ~BackendServer();
    void run();

    // Now public
    std::string process_json_post(const httplib::Request& req, mongocxx::collection& collection);
    std::string process_binary_post(const httplib::Request& req, mongocxx::collection& collection);
    
private:
    void initialize_database();
    //std::string process_binary_post(const httplib::Request& req, mongocxx::collection& collection);

    mongocxx::client client;
    mongocxx::database db;
    mongocxx::collection collection;
    httplib::Server svr;
    DatabaseConfig config;
};


#endif // BACKEND_H






// #ifndef BACKEND_H
// #define BACKEND_H

// #include "httplib.h"
// #include <mongocxx/client.hpp>
// #include <mongocxx/instance.hpp>
// #include <bsoncxx/json.hpp>
// #include <bsoncxx/types.hpp>
// #include <bsoncxx/builder/stream/document.hpp>
// #include <bsoncxx/builder/basic/document.hpp>
// #include <string>
// #include <chrono>

// // MongoDB access details
// struct DatabaseConfig {
//     std::string uri = "mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0";
//     std::string database_name = "meshData";
//     std::string collection_name = "binary";
// };

// // Singleton for MongoDB instance management
// class MongoDBInstance {
// public:
//     static mongocxx::instance& get_instance() {
//         static mongocxx::instance instance;  // This instance is created only once.
//         return instance;
//     }

// private:
//     MongoDBInstance() {}  // Constructor is private to prevent external instantiation.
// };

// // Initialize MongoDB client
// void initialize_database(mongocxx::client& client, const DatabaseConfig& config);

// // Handle JSON post request
// std::string process_json_post(const httplib::Request& req, mongocxx::collection& collection);

// // Handle binary post request
// std::string process_binary_post(const httplib::Request& req, mongocxx::collection& collection);

// #endif // BACKEND_H

