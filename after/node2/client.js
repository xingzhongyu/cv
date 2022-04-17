let express=require("express");
var ws = require("ws");
let request=require("request");
let sync_get=function(params){
    return new Promise(function(resolve,reject){
        request.get(params,function(req,res){
            resolve(res)
        })
    })
}



let app=express();
const port=9000;
const hostname='10.27.136.211';
app.get("hello",(req,res)=>{
    res.json({
        code:0,
        msg:'success'
    })
})
var server=app.listen(port,hostname,(req,res)=>{
    console.log(`Server running at http://${hostname}:${port}/`)
})
