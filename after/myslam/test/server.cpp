#include "workflow/WFFacilities.h"
#include <csignal>
#include "wfrest/HttpServer.h"
#include "wfrest/json.hpp"
#include <vector>
#include "run_vo2.h"
// #include <jsoncpp/json/json.h>
using namespace wfrest;

int main()
{
    HttpServer svr;

    // curl -v http://ip:port/hello
    svr.GET("/hello", [](const HttpReq *req, HttpResp *resp)
    {
        resp->String("World\n");


    });
  svr.GET("/change", [](const HttpReq *req, HttpResp *resp)
    {
        changeInit();

        resp->String("test\n");
    });
    // curl -v http://ip:port/data
    svr.GET("/data", [](const HttpReq *req, HttpResp *resp)
    {
        std::string str = "Hello world";
        resp->String(std::move(str));
    });

    // curl -v http://ip:port/post -d 'post hello world'
   svr.POST("/test3", [](const HttpReq *req, HttpResp *resp)
    {
        if (req->content_type() != APPLICATION_JSON)
        {
            resp->String("NOT APPLICATION_JSON");
            return;
        }
        // fprintf(stderr, "Json : %s", req->json().dump(1).c_str());
        // string temp=
        // wfrest::Json json;
        // json["test3"]=req->json()["filePath1"];
        // resp->Json(json);
        string filePath1=req->json()["filePath1"];
        string filePath2=req->json()["filePath2"];
        string ans=readImg(filePath1,filePath2);
        resp->String(ans);
    });

    if (svr.start(8888) == 0)
    {
        getchar();
        svr.stop();
    } else
    {
        fprintf(stderr, "Cannot start server");
        exit(1);
    }
    return 0;
}
