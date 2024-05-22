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
    get_instance();
    mongocxx::uri uri("mongodb+srv://mkchappe:passwordOne@cluster0.4jfwepn.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0");
    client = mongocxx::client(uri);
    db = client["meshData"];
    collection = db["binary"];
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

void MongoDBClient::downloadMeshData(const std::string &input)
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
            return;
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
            auto filename = "output.stl"; // Change to your desired path

            ofstream file(filename, ios::out | ios::binary);
            if (file)
            {
                file.write(reinterpret_cast<const char *>(binary_data.bytes), binary_data.size);
                file.close();
                cout << "Mesh data saved as " << filename << endl;
            }
            else
            {
                cout << "Failed to save mesh data." << endl;
            }
        }
        else
        {
            cout << "No matching document found." << endl;
        }
    }
}
