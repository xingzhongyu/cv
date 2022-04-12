#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#define NAPI_EXPERIMENTAL
#include <node_api.h>
// #include <napi.h>
#include <v8.h>
using namespace v8;
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv4/opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include "sophus/se3.h"

// using Sophus::SE3;

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>
// using namespace Eigen;

using namespace std;

#include "map.h"
struct Argv{
    Argv(char* filepath,double* data):_filePath(filepath),_data(data){}
    char* _filePath;
    double* _data;
};
typedef struct
{
    napi_async_work work;          // 保存线程任务的
    napi_threadsafe_function tsfn; // 保存回调函数的
    Argv* argv;
    // int argv[2];
} Addon;

// int a,b;
/** 调试报错用的 */
static void catch_err(napi_env env, napi_status status)
{
    if (status != napi_ok)
    {
        const napi_extended_error_info *error_info = NULL;
        napi_get_last_error_info(env, &error_info);
        printf("%s\n", error_info->error_message);
        exit(0);
    }
}

/** 调用 js-callback 用的 */
static void call_js(napi_env env, napi_value js_cb, void *context, void *data)
{
    (void)context; // 用不到它
    int sum2=3;
    Argv *d = (Argv *)data;
    
    cout<<d->_filePath<<endl;
    // cout<<d->_data[0]<<endl;




    vector<double> vector;
    startCV(d->_filePath,d->_data,vector);

    // sum2=sum(d[0],d[1]);

    // cv::Mat img;
    // napi_get_value_int32(env,d[1],&a);
    // printf("%d--",a);

    napi_value arg;
    napi_create_int32(env, sum2, &arg);
    
    
    napi_value argv[] = {arg};
    if (env != NULL)
    {
        napi_value undefined;

        napi_get_undefined(env, &undefined); // 创建一个 js 的 undefined
        catch_err(env, napi_call_function(env,
                                          undefined, // js 回调的 this 对象
                                          js_cb,     // js 回调函数句柄
                                          1,         // js 回调函数接受参数个数

                                          argv,   // js 回调函数参数数组
                                          NULL)); // js 回调函数中如果有 retrun，将会被 result 接受到，NULL 代表忽略
    }
}

/** 执行线程 */
static void execute_work(napi_env env, void *data)
{
    Addon *addon = (Addon *)data;

    // 拿到 js-callback 函数
    catch_err(env, napi_acquire_threadsafe_function(addon->tsfn));

    // 延迟四秒执行
    // sleep(4);

    //   napi_value  arg;

    //  catch_err(env, napi_create_string_utf8(env,
    //     "test",
    //     NAPI_AUTO_LENGTH,
    //     &arg));
    // napi_value argv[]={arg};

    // 调用 js-callback 函数
    catch_err(env, napi_call_threadsafe_function(
                       addon->tsfn,          // js-callback 函数
                       addon->argv,          // call_js 的第四个参数
                       napi_tsfn_blocking)); // 阻塞模式调用

    // 释放句柄
    catch_err(env, napi_release_threadsafe_function(addon->tsfn, napi_tsfn_release));
}

/** 线程执行完成 */
static void work_complete(napi_env env, napi_status status, void *data)
{
    Addon *addon = (Addon *)data;

    // 释放句柄
    catch_err(env, napi_release_threadsafe_function(addon->tsfn, napi_tsfn_release));

    // 回收任务
    catch_err(env, napi_delete_async_work(env, addon->work));

    addon->work = NULL;
    addon->tsfn = NULL;
}

/**
 * start_thread 启动线程
 * 关于 static 关键字用不用都行的，官方的例子有用到 static
 * 以我 js 的能力我猜的可能是开多个线程下，可以公用一个函数，节约内存开销 (欢迎大神来讨论 😭)
 */
static napi_value start_thread(napi_env env, napi_callback_info info)
{
    size_t argc = 3;      // js 传进来的参数个数
    napi_value js_cb;     // js 传进来的回调函数
    napi_value work_name; // 给线程起个名字
    Addon *addon;         // “实例化” 结构体 (个人理解是取出了传进来的 js-cabllback 地址指针，期待大神来讨论 😭)
    napi_status sts;      // 程序执行状态
    napi_value args[3];
    sts = napi_get_cb_info(
        env,              // 执行上下文，可以理解为 js 那个 “事件环”
        info,             // 上下文信息
        &argc,            // 收到参数的个数
        args,             // 接收 js 参数
        NULL,             // 接收 js 的 this 对象
        (void **)(&addon) // 取得 js 传进来的 callback 的指针地址
    );
    js_cb = args[2];

    size_t read1;
    size_t read2;
    char a[128];
    char b[512];
    
    size_t b_size;
    napi_get_value_string_utf8(env,args[0],a,sizeof(a),&read1);
    // assert(0);
    napi_get_value_string_utf8(env,args[1],b,sizeof(b),&read2);
    string str=b;
    double data[7];
    int i=0;

    istringstream iss(str);
    string token;
    while (getline(iss,token,','))
    {
        data[i]=stod(token);
        i++;
    }
    addon->argv=new Argv(a,data);
    

    
    


    // printf("%d--",a);
    catch_err(env, sts);

    // 打酱油的 ^_^
    assert(addon->work == NULL && "Only one work item must exist at a time");

    // 创建线程名字
    catch_err(env, napi_create_string_utf8(env,
                                           "N-API Thread-safe Call from Async Work Item",
                                           NAPI_AUTO_LENGTH,
                                           &work_name));

    // 把 js function 变成任意线程都可以执行的函数
    // 酱紫我们就可以在开出来的子线程中调用它咯
    sts = napi_create_threadsafe_function(env,

                                          // 其他线程的 js 函数
                                          // call_js 的第二个参数
                                          // 也就是我们 addon.start(function) 传进来的 function
                                          js_cb,

                                          // 可能传递给一些异步任务async_hooks钩子传递初始化数据 (期待大神讨论 😊)
                                          // 个人理解 N-API 中的 async 指的就是多线程任务
                                          // 一个线程任务，在 N-API 中由 async work 调用
                                          NULL,

                                          work_name,       // 给线程起个名字，给 async_hooks 钩子提供一个标识符
                                          0,               // (官网直译)最大线程队列数量，0 代表没限制
                                          1,               // (官网直译)初始化线程数量，其中包括主线程
                                          NULL,            // (官网直译)线程之间可以传递数据(官网直译)
                                          NULL,            // (官网直译)线程之间可以传递函数，函数注销时候被调用
                                          NULL,            // (官网直译)附加给函数的执行上下文，应该就是
                                          call_js,         // call_js 的第三个参数
                                          &(addon->tsfn)); // js 传进来的函数，可以理解为真实的 js 函数所在内存地址 (期待大神讨论 😊)
    catch_err(env, sts);

    // 负责执行上面创建的函数
    sts = napi_create_async_work(env,
                                 NULL,            // 可能传递 async_hooks 一些初始化数据
                                 work_name,       // 给线程起个名字，给 async_hooks 钩子提供一个标识符
                                 execute_work,    // 线程执行时候执行的函数 (与主线程并行执行)
                                 work_complete,   // 线程执行完时候的回调
                                 addon,           // 既 execute_work、work_complete 中的 void* data
                                 &(addon->work)); // 线程句柄
    catch_err(env, sts);

    // 将线程放到待执行队列中
    sts = napi_queue_async_work(env,
                                // 要执行线程的句柄
                                addon->work);
    catch_err(env, sts);

    return NULL; // 这个貌似是返回给 js-callback 的返回值
}

napi_value init(napi_env env, napi_value exports)
{
    // 这里等价于 const obj = new Object();
    // 这回知道面向对象是咋来的了吧 😁
    // 类的本质就是“结构体”演化而来的，new(开辟堆内存空间) 关键字是 malloc(申请内存空间) 变种过来的
    Addon *addon = (Addon *)malloc(sizeof(*addon));

    // 等价于 obj.work = null;
    addon->work = NULL;

    // 个人 js 水平有限，Object 类研究的不深
    // 可以说，精通 Object 类的小伙伴可以自己想想咯，反正给对象挂一个属性、函数需要的东东，都在这里了
    // 相等于 const fun = () => {}, attr = 'Hello';
    napi_property_descriptor desc = {
        "start",      // 属性名称
        NULL,         // -- 没想明白
        start_thread, // 函数体
        NULL,         // 属性 getter
        NULL,         // 属性 setter
        NULL,         // 属性描述符
        napi_default,
        addon // (官网直译)也可以写 NULL，调用 getter 时候返回的数据
    };
    // 相当于 const obj = { fun, attr };
    napi_define_properties(env, exports, 1, &desc); // 将属性挂载到 exports 上面

    return exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, init)

// 配合另一种导出模块用的
// static void addon_getting_unloaded(napi_env env, void* data, void* hint) {
//  Addon* addon = (Addon*)data;
//  assert(addon->work == NULL && "No work in progress at module unload");
//  free(addon);
//}

// 另一种导出模块的宏定义(有空再研究🙃)
// NAPI_MODULE_INIT() {
//  Addon* addon = (Addon*)malloc(sizeof(*addon));
//  addon->work = NULL;
//
//  napi_property_descriptor desc = {
//      "start",
//      NULL,
//      start_thread,
//      NULL,
//      NULL,
//      NULL,
//      napi_default,
//      addon
//  };
//
//  napi_define_properties(env, exports, 1, &desc);
//  napi_wrap(env, exports, addon, addon_getting_unloaded, NULL, NULL);
//  // return exports; // 可写可不写
//}
