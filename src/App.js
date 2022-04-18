import './App.css';
import UploadImg from './upload/upload'
import ImgShow from './ImgShow/ImgShow';
import { Button, Drawer, Grid } from '@mui/material';
import { useState } from 'react';
import Upload2 from './upload2/upload2';
import axios from 'axios';
import Upload3 from './upload3/upload3';
import Point_show from './point_show/point_show';







function App() {
  const [pose, setPose] = useState(1);
  const [data, setData] = useState([])
  const [points, set_points] = useState([])
  const handleClick = (val) => {
    if (pose !== val) {
      setPose(val)
    }
  }
  const handleRece = (val) => {
    // console.log(val)
    setData(val)
  }
  const handle_points = (val) => {
    set_points(val);
  }
  const handleInit = () => {
    let config = {
      url: 'http://10.27.136.63:10001/changeInit',
      method: 'get'
    }
    axios(config).then(function (res) {
      if (res.statusCode === 200) {
        console.log(res)
      }
      window.location.reload(true)
    }).catch(function (err) {
      console.log(err)
    })


  }
  const handle_init=()=>{
    let config={
      url:'http://10.27.136.211:3000/change',
      method:'get'
    }
    axios(config).then(function(res){
      // console.log(res)
      if(res.statusCode===200){
        console.log(res)
      }
      window.location.reload(true)
    }).catch(function(err){
      console.log(err)
    })
  }


  let show;
  if (pose === 1) {
    show = <UploadImg handleRece={handleRece} />
  } else if (pose === 0) {
    show = <Upload2 handleRece={handleRece} />
  } else if (pose === 2) {
    show = <Upload3 handle_points={handle_points}></Upload3>
  }
  let imgShow;
  if (pose === 0 || pose === 1) {
    imgShow = <ImgShow data={data} />
  } else {
    imgShow = <Point_show points={points} />
  }

  return (
    <div className="App">


      <Grid container direction="row" justifyContent="flex-start" alignItems="center" spacing={2}>
        <Grid item xs={3}>
          <Drawer variant='permanent' anchor='left' sx={{
            width: 240, flexShrink: 0,
            '& .MuiDrawer-paper': {
              width: 240,

              boxSizing: 'border-box',
            }
          }}>
            <Button onClick={() => handleClick(1)} variant={pose == 1 ? "contained" : "text"}>有位姿</Button>
            <Button onClick={() => handleClick(0)} variant={pose == 0 ? "contained" : "text"}>无位姿</Button>
            <Button onClick={() => handleClick(2)} variant={pose == 2 ? "contained" : "text"} >彩色图加深度图</Button>


            <Button onClick={() => { handleInit() }} color="error">撤销</Button>
            <Button onClick={()=> handle_init()} color="error">撤销深度图</Button>

          </Drawer>
        </Grid>
        <Grid item xs={3}>
          <div>
            {show}

          </div>
        </Grid>
        <Grid item xs={5}>
          {imgShow}
        </Grid>

      </Grid>
    </div>
  );
}

export default App;

