import {  useState } from "react";
import {Button,  CircularProgress,  Stack, TextField} from '@mui/material'
import axios from "axios";

var fileSaver=require('file-saver')

function Upload(props){
    const [imgShow,setImgShow]=useState(false)
    const [file,setFile]=useState({})
    const [imgSrc,setImgSrc]=useState('')
    const [pose,setPose]=useState('')
    const [load,setLoad]=useState('none')
    const handeleImageChange=(e)=>{
        let file=e.target.files[0];
        e.preventDefault();
        let reader=new FileReader();
        reader.onloadend=()=>{
            setFile(e.target.files[0])
            
            setImgShow(true)
            setImgSrc(reader.result)
        }
        reader.readAsDataURL(file)
    }
    const handlePoseChange=(e)=>{
        e.preventDefault()
        setPose(e.target.value)
    }
    const handleSubmit=()=>{
        setLoad('block')
        let formData=new FormData();
        formData.append('imgfile',file)
        formData.append('pose',pose)
        let config={
            method:'post',
            url: 'http://10.27.136.63:10001/upload/img',
            headers:{
                'Content-Type': 'multipart/form-data'
            },
            data:formData
        }
        axios(config).then(function(res){
            let data=res.data

            // var blob=new Blob([JSON.stringify(data)],{type: "text/plain;charset=utf-8"})
            // fileSaver.saveAs(blob,'res.json')
            props.handleRece(data.ans)
            setLoad('none')
            // console.log(data)

            // console.log(data.ans)

        }).catch(function(err){
            console.log(err)
        })
    }
    return (
        <div>
      <Stack spacing={2}  alignItems="center">
          
          <p style={{margin:'0'}}>上传图片</p>
        <input id='ava' type='file' style={{display:'none'}} onChange={(e)=>handeleImageChange(e)}></input>
        {imgShow?
        <label htmlFor="ava"><img src={imgSrc} style={{width:'80px',height:'80px'}} alt=''></img></label>
        :<label style={{color:"#1890FF",border:"1px dashed #1890FF",padding:'3px 10px '}} htmlFor="ava">+点击上传图片</label>}
        <TextField variant="standard" helperText="位姿数据为四元数加向量7位,以逗号隔开" onChange={(e)=>handlePoseChange(e)}></TextField>
        <Button variant='contained' onClick={()=>handleSubmit()}>确定提交</Button>
        <CircularProgress style={{display:load}}/>
      
      </Stack>
      </div>
    )
}
export default Upload