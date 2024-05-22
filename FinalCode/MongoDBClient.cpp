#include "MongoDBClient.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <optional>


using namespace std;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;

MongoDBClient::MongoDBClient()
{
    try {
        get_instance();
        mongocxx::uri uri("mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0");
        client = mongocxx::client(uri);
        db = client["meshData"];
        collection = db["binary"];
    }
    catch (const mongocxx::exception& me) {  // Handle MongoDB-specific exceptions
        std::cerr << "MongoDB connection error: " << me.what() << std::endl;
    }
}

void MongoDBClient::displayDocuments()
{
    cout << "Collections in database meshData:" << endl;
    cout << "Documents in collection binary:" << endl;

    auto cursor = collection.find({});
    for (auto &&doc : cursor)
    {
        if (doc["_id"] && doc["timestamp"])
        {
            auto id = doc["_id"].get_oid().value.to_string();
            auto millisec_since_epoch = doc["timestamp"].get_date().value;
            auto timepoint = std::chrono::system_clock::time_point(std::chrono::milliseconds(millisec_since_epoch));
            auto tt = std::chrono::system_clock::to_time_t(timepoint);
            auto tm = *std::localtime(&tt);
            std::stringstream ss;
            ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
            cout << id << " - " << ss.str() << endl;
        }
    }
}

std::optional<std::vector<uint8_t>> MongoDBClient::downloadMeshData(const std::string &input)
{
    bsoncxx::stdx::optional<bsoncxx::types::b_oid> oid;
    bsoncxx::stdx::optional<bsoncxx::types::b_date> date;

    try
    {
        oid = bsoncxx::types::b_oid{bsoncxx::oid(input)};
    }
    catch (...)
    {
        try
        {
            auto timestamp = std::stoi(input);
            date = bsoncxx::types::b_date(std::chrono::milliseconds(timestamp));
        }
        catch (...)
        {
            cout << "Invalid input. Please enter a valid ID or Timestamp." << endl;
            return std::nullopt;  // No data found
        }
    }

    std::optional<bsoncxx::document::value> filter_doc;

    if (oid)
    {
        filter_doc.emplace(document{} << "_id" << *oid << finalize);
    }
    else if (date)
    {
        filter_doc.emplace(document{} << "timestamp" << *date << finalize);
    }

    if (filter_doc)
    {
        auto result = collection.find_one(filter_doc->view());
        if (result)
        {
            auto binary_data = result->view()["data"].get_binary();
            std::vector<uint8_t> data(binary_data.bytes, binary_data.bytes + binary_data.size);
            return data;  // Successfully found and returning data
        }
        else
        {
            cout << "No matching document found." << endl;
            return std::nullopt;  // No data found
        }
    }
    return std::nullopt;  // No data found
}


void MongoDBClient::uploadBinarySTLtoMongoDB(const std::vector<char>& binarySTL, const std::string collectionName) 
{
    //change to edited collection
    auto collection = db[collectionName];

    bsoncxx::types::b_binary binary_data{
        bsoncxx::binary_sub_type::k_binary, // Binary subtype
        static_cast<uint32_t>(binarySTL.size()),
        reinterpret_cast<const uint8_t*>(binarySTL.data())
    };

    // Current system time as a timestamp
    auto now = std::chrono::system_clock::now();

    bsoncxx::builder::stream::document document{};
    document << "name" << "STL Model"
             << "data" << binary_data
             << "timestamp" << bsoncxx::types::b_date(now); // Pass the time_point directly

    collection.insert_one(document.view());
}
