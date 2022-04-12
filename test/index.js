const addon = require('../build/libobject_node2.node');
function startCV(filePath,arr){
  let points;
  console.log(filePath)
  // console.log(arr)
  let str=JSON.stringify(arr);
  const buffer=str.substring(1,str.length-1);
  // console.log(buffer);
  addon.startCV(filePath,buffer,(data)=>{
    points=data;
  })
  console.log(points.length);

  return points;

}

function startCV2(filePath){
  let points;
  console.log(filePath)
  addon.startCV2(filePath,(data)=>{
    points=data;
  })
  return points;
}
function changeCV(){
  addon.changeCV();
}
module.exports={startCV,startCV2,changeCV};



// let second = 0;
// let cb_exucted = false;

// let pose=[1.086410 ,4.766730 ,-1.449960 ,0.789455 ,0.051299 ,-0.000779 ,0.611661].reverse();
// let str=JSON.stringify(pose);
// const buffer=str.substring(1,str.length-1);
// console.log(JSON.stringify(pose));

// addon.start("/home/xzy/Downloads/test_data/images/scene_000.png",buffer,(data)=> {


//   cb_exucted = true;

//   console.log(data);

  
//   console.log(`Asynchronous callback exectued.`);
// });

// const t = setInterval(() => {
//   if (cb_exucted) {
//     clearTimeout(t);
//   }
//   console.log(`After ${++second} seconds.`);
// }, 1000);