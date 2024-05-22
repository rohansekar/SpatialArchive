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

class MongoDBClient
{
public:
    MongoDBClient();
    void displayDocuments();
    void downloadMeshData(const std::string &input);

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
