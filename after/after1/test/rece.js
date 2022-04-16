const start=require("./index")
var express=require('express');
let body_parser=require('body-parser');
var app=express();


const hostname = '127.0.0.1'; //此处为便于测试，以后修改
const port = 3001
app.use(body_parser.json())
app.use(body_parser.urlencoded({ extended: true }))



app.get("/hello",(req,res)=>{
    res.json({
        code:0,
        msg:'hello'
    })
})


// app.use(jsonParser);
app.post("/test", (req, res) => {
    
    console.log(req.body)
    let points=start.startCV(req.body.filePath,req.body.pose);

    res.json({
        code:0,
        msg:points
    })

})
app.post("/test2",(req,res)=>{
    let points=start.startCV2(req.body.filePath);

    res.json({
        code:0,
        msg:points
    })
}) 
app.get("/change",(req,res)=>{
    start.changeCV();
    res.json({
        code:0,
        msg:'success'
    })
})
var server=app.listen(port,hostname,()=>{
    console.log(`Server running at http://${hostname}:${port}/`)
})


// var request=require('request');
// var fs=require('fs');
// request('http://10.27.193.94:10001/readImg?filePath=uploads/1649211784198-cdfb263f956007331ae42b89338753d6_720w.png',function(error,response,body){
//     if(!error&&response.statusCode==200){
//         // console.log(response)
//         // console.log(body.toString('base64').length);
//         //  buffer=Buffer.from(body,'base64');

//         // console.log(buffer);




//         // buffer[0]=0x89;
//         // console.log(buffer)
//         fs.writeFile('./test1.png',Buffer.from(body,'base64'),(err)=>{
//             console.log(err);
//         });
//         // console.log(response);


//     }
// })