#ifndef MONGODB_MANAGER_H
#define MONGODB_MANAGER_H

#include <string>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/exception/exception.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/instance.hpp>
#include <optional>
#include <vector>


class MongoDBClient
{
public:
    MongoDBClient();
    void displayDocuments();
    std::optional<std::vector<uint8_t>> downloadMeshData(const std::string &input);
    void uploadBinarySTLtoMongoDB(const std::vector<char>& binarySTL, const std::string collectionName);

private:
    static mongocxx::instance &get_instance()
    {
        static mongocxx::instance instance{};
        return instance;
    } 
    mongocxx::client client;
    mongocxx::database db;
    mongocxx::collection collection;

};

#endif // MONGODB_MANAGER_H
