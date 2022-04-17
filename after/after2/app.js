"use strict";
const hostname = '10.27.136.211';
const port = 3000
exports.__esmodule = true;
var express = require("express");
var multer = require("multer");
var app = express();
let fs = require('fs');
let path = require('path');
var bodyParse = require("body-parser");
let request = require("request");
// const res = require("express/lib/response");
// const { resolve } = require("path");


// var urls=fs.readdir("./uploads")
var storage = multer.diskStorage({
    destination: function (req, file, cb) {
        cb(null, "./uploads");
    },
    filename: function (req, file, cb) {
        cb(null, `${Date.now()}-${file.originalname}`)
    }
})
var upload = multer({ storage: storage });
// var imgBaseUrl = "../";
app.use(bodyParse.json());
app.use(bodyParse.urlencoded({ extended: false }));
app.use(express.static('uploads'));

app.all('*', function (req, res, next) {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Headers', 'Content-Type, Content-Length, Authorization, Accept, X-Requested-With , yourHeaderFeild');
    res.header('Access-Control-Allow-Methods', 'PUT, POST, GET, DELETE, OPTIONS');

    if (req.method == 'OPTIONS') {
        res.send(200); //让options请求快速返回/
    }
    else {
        next();
    }
});
let sync_post = function (params) {
    return new Promise(function (resolve, reject) {
        request.post(params, function (req, res) {
            resolve(res)
        })
    });
}
let sync_get=function(params){
    return new Promise(function(resolve,reject){
        request.get(params,function(req,res){
            resolve(res)
        })
    })
}
app.get('/changeInit',async function(req,res){
    let ans=await sync_get({
        url:'http://127.0.0.1:3001/change',
        method:'get'
    })
    res.end(JSON.stringify(ans));
})
app.get("/change",async function(req,res){
    let ans=await sync_get({
        url:'http://127.0.0.1:8888/change',
        method:'get'
    })
})
app.post('/upload/img', upload.array('imgfile', 2), async function (req, res) {

    // console.log(req);
    var files = req.files;
    let pose = req.body.pose.split(',').map((item) => { return parseFloat(item) });
    // console.log(pose[0])
    var result = {};
    console.log(files[0]);
    if (!files[0]) {
        result.code = 1;
        result.errMsg = '上传失败';
    } else {
        result.code = 0;
        let filePath = path.join('/home/xzy/object_node3', files[0].path);
        let body = {
            filePath: filePath,
            pose: pose
        }
        let ans= await sync_post({
            url: 'http://127.0.0.1:3001/test',
            method: 'POST',
            json: true,
            headers: {
                "content-type": "application/json",
            },
            body: body
        })
        // console.log("test");
        // console.log(ans)
        result.ans=ans.body.msg;

        result.msg = '上传成功';


        // url.push(result.data);
    }
    res.end(JSON.stringify(result))

})
app.post('/img3',upload.array("imgfile",2),async function(req,res){
    var files=req.files;
    var result={};
    if(!files[0]){
        result.code = 1;
        result.errMsg = '上传失败';
    }else{
        result.code=0;
        
        let filePath1=path.join('/home/guojiawei/cv/cv2/cv/after/after2',files[0].path);
        let filePath2=path.join('/home/guojiawei/cv/cv2/cv/after/after2',files[1].path);
        let body={
            filePath1:filePath1,
            filePath2:filePath2
        }
        let ans=await sync_post({
            url: 'http://127.0.0.1:8888/test3',
            method: 'POST',
            json: true,
            headers: {
                "content-type": "application/json",
            },
            body: body
        })
        // fs.writeFile("./pointx.txt",ans.body,err=>{
        //     if (err) {
        //         console.error(err)
        //         return;
        //     }
        // })
        result.ans=ans;

        result.msg='上传成功'
    }
    res.end(JSON.stringify(result));
})
app.post('/upload/img2',upload.array('imgfile',2),async function(req,res){
    var files=req.files;
    var result = {};
    if (!files[0]) {
        result.code = 1;
        result.errMsg = '上传失败';
    } else {
        result.code = 0;
        let filePath = path.join('/home/xzy/object_node3', files[0].path);
        let body={
            filePath:filePath
        }
        let ans=await sync_post({
            url: 'http://127.0.0.1:3001/test2',
            method: 'POST',
            json: true,
            headers: {
                "content-type": "application/json",
            },
            body: body
        })
        result.ans=ans.body.msg
        result.msg='上传成功'
    }
    res.end(JSON.stringify(result));
})
function readImg(path) {
    return new Promise(resolve => {
        fs.readFile(path, 'base64', function (err, file) {
            if (err) {
                console.log(err);
            } else {
                // res.write(file,'base64');
                // console.log(file.toString('base64').length)
                // let buffer=Buffer.from(file,'base64').toString('base64');
                // console.log(buffer)
                // console.log(Buffer.from(buffer.toString('base64'),'base64'))
                // fs.writeFile('./test1.png',Buffer.from(buffer,'base64'),function(error){
                //     console.log(error)
                // })
                // fs.writeFile('./test1.png',Buffer.from(file,'base64'),function(err){
                //     console.log(err);
                // })
                // console.log(1)
                // console.log(file)
                resolve(file);
                // res.end()
                // console.log(res)
            }


        })
    })
    // console.log(res)
}
app.get('/readImg', async function (req, res) {
    // console.log(req.query)

    let filePath = path.join('/home/xzy/object_node3', req.query.filePath);
    // console.log(filePath);
    // res.writeHead(200,{'Content-Type' : 'image/jpeg'});
    let buffer = await readImg(filePath);
    // console.log(2)
    res.send(buffer);
    res.end();

})

async function readImgs(paths) {
    let filePath1 = path.join('/home/xzy/object_node3', paths[0]);
    let filePath2 = path.join('/home/xzy/object_node3', paths[0]);
    let buffer1 = await readImg(filePath1);
    let buffer2 = await readImg(filePath2);
    return { buffer1, buffer2 };
}

app.get('/test', (req, res) => {
    res.statusCode = 200
    res.setHeader('Content-Type', 'text/plain')
    res.end('Hello World\n')
})


var server = app.listen(port, hostname, () => {
    console.log(`Server running at http://${hostname}:${port}/`)
})


