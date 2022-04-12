import './App.css';
import UploadImg from './upload/upload'
import ImgShow from './ImgShow/ImgShow';
import { Button, Drawer, Grid } from '@mui/material';
import { useState } from 'react';
import Upload2 from './upload2/upload2';
import axios from 'axios';








function App() {
  const [pose, setPose] = useState(true);
  const [data,setData]=useState([])
  const handleClick = (val) => {
    if (pose !== val) {
      setPose(val)
    }
  }
  const handleRece=(val)=>{
    // console.log(val)
    setData(val)
  }
  const handleInit = () => {
    let config = {
      url: 'http://10.27.136.63:10001/changeInit',
      method: 'get'
    }
    axios(config).then(function (res) {
      if(res.statusCode==200){
        console.log(res)
      }
      window.location.reload(true)
    }).catch(function (err) {
      console.log(err)
    })

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
            <Button onClick={() => handleClick(true)} variant={pose ? "contained" : "text"}>有位姿</Button>
            <Button onClick={() => handleClick(false)} variant={pose ? "text" : "contained"}>无位姿</Button>
            <Button onClick={() => { handleInit() }} color="error">撤销</Button>

          </Drawer>
        </Grid>
        <Grid item xs={3}>
          <div>

            {pose ? <UploadImg handleRece={handleRece}/> : <Upload2 handleRece={handleRece}/>}
          </div>
        </Grid>
        <Grid item xs={5}>
          <ImgShow data={data}/>
        </Grid>

      </Grid>
    </div>
  );
}

export default App;

