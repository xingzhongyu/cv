#include <stdio.h>
#include "map.h"
#include <string.h>

#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#define NAPI_EXPERIMENTAL
#include <node_api.h>

using namespace std;

static napi_value RunCallback(napi_env env, const napi_callback_info info) {
  napi_status status;

  size_t argc = 3;
  napi_value args[3];
  status = napi_get_cb_info(env, info, &argc, args, NULL, NULL);
  assert(status == napi_ok);

  napi_value cb = args[2];

  
   size_t read1;
    size_t read2;
    char a[128];
    char b[128];
     napi_get_value_string_utf8(env,args[0],a,sizeof(a),&read1);
    // assert(0);
    napi_get_value_string_utf8(env,args[1],b,sizeof(b),&read2);
    string str=b;
    double data[7];
    int i=0;
    // cout<<b<<endl;
    istringstream iss(str);
    string token;
    while (getline(iss,token,','))
    {
        data[i]=stod(token);
        i++;
    }


    
    // for (size_t j = 0; j < 7; j++)
    // {
    //   cout<<data[j]<<" ";
    // }
    // cout<<endl;
    
      vector<unsigned char> v;
string ans;
    ans=startCV(a,data);

  // vector<unsigned char>::iterator ite=v.begin();
  // for(;ite!=v.end();ite++){
  //   cout<<*ite<<" ";
  // }
  // cout<<endl;
  // napi_value vs;
  // status=napi_create_array(env,&vs);
  // napi_value p;
  // int j=0;
  // vector<unsigned char>::iterator ite=v.begin();
  // for(;ite!=v.end();ite++){
  //     status=napi_create_int32(env,*ite,&p);
  //     status=napi_set_element(env,vs,j,p);
  //     j++;
  // }
  napi_value argv[1];
  // status = napi_create_string_utf8(env, "hello world", NAPI_AUTO_LENGTH, argv);
  // argv[0]=vs;
  status=napi_create_string_utf8(env,ans.c_str(),NAPI_AUTO_LENGTH,argv);
  assert(status == napi_ok);

  napi_value global;
  status = napi_get_global(env, &global);
  assert(status == napi_ok);

  napi_value result;
  status = napi_call_function(env, global, cb, 1, argv, &result);
  assert(status == napi_ok);

  return NULL;
}
static napi_value RunCallback2(napi_env env,napi_callback_info info){
  napi_status status;
  size_t argc=2;
  napi_value args[2];
  status=napi_get_cb_info(env,info,&argc,args,NULL,NULL);
  assert(status==napi_ok);
  napi_value cb=args[1];
  char a[128];
  size_t read;
  napi_get_value_string_utf8(env,args[0],a,sizeof(a),&read);
  // Mat* gray = new Mat(480, 640, CV_8UC1);
  // vector<unsigned char> v;
  string ans;
  ans=startCV2(a);

  // if(gray.isContinuous()){
  //   v=gray.data;
  // }
  // napi_value vs;
  // status=napi_create_array(env,&vs);
  // napi_value p;
  // int j=0;
  // vector<unsigned char >::iterator ite=v.begin();
  // for(;ite!=v.end();ite++){
  //     status=napi_create_int32(env,*ite,&p);
  //     status=napi_set_element(env,vs,j,p);
  //     j++;
  // }
  
  napi_value argv[1];
  // status = napi_create_string_utf8(env, "hello world", NAPI_AUTO_LENGTH, argv);
  // argv[0]=vs;
  status=napi_create_string_utf8(env,ans.c_str(),NAPI_AUTO_LENGTH,argv);
  assert(status == napi_ok);

  napi_value global;
  status = napi_get_global(env, &global);
  assert(status == napi_ok);

  napi_value result;
  status = napi_call_function(env, global, cb, 1, argv, &result);
  assert(status == napi_ok);
  return NULL;
}
static napi_value changeCV(napi_env env,napi_callback_info info){
  changeInit();
  return NULL;
}
static napi_value Init(napi_env env, napi_value exports) {
  napi_value obj;
  napi_create_object(env,&obj);
  napi_property_descriptor descs[3]; 
  napi_property_descriptor desc1={"startCV",NULL,RunCallback,NULL,NULL,NULL,napi_default,NULL};
  napi_property_descriptor desc2={"startCV2",NULL,RunCallback2,NULL,NULL,NULL,napi_default,NULL};
  napi_property_descriptor desc3={"changeCV",NULL,changeCV,NULL,NULL,NULL,napi_default,NULL};
  descs[0]=desc1;
  descs[1]=desc2;
  descs[2]=desc3;
  napi_define_properties(env,obj,3,descs);
  // napi_value new_exports;
  // napi_status status = napi_create_function(
  //     env, "", NAPI_AUTO_LENGTH, RunCallback, NULL, &new_exports);
  assert(status == napi_ok);
  return obj;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)