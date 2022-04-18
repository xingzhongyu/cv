import { Stack,Button, CircularProgress } from "@mui/material";
import axios from "axios";
import { useState } from "react";


function Upload2(props){
    const [imgShow,setImgShow]=useState(false)
    const [file,setFile]=useState({})
    const [imgSrc,setImgSrc]=useState('')
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
    const handleSubmit=()=>{
        setLoad('block')
        let formData=new FormData();
        formData.append('imgfile',file)
        let config={
            method:'post',
            url: 'http://10.27.193.94:10001/upload/img2',
            headers:{
                'Content-Type': 'multipart/form-data'
            },
            data:formData
        }
        axios(config).then(function(res){
            let data=res.data;
            props.handleRece(data.ans)
            setLoad('none')

        }).catch(function(err){
            console.log(err)
        })

    }
    return(
        <div>
        <Stack spacing={2}  alignItems="center">
          
          <p style={{margin:'0'}}>上传图片</p>
        <input id='ava' type='file' style={{display:'none'}} onChange={(e)=>handeleImageChange(e)}></input>
        {imgShow?
        <label htmlFor="ava"><img src={imgSrc} style={{width:'80px',height:'80px'}} alt=''></img></label>
        :<label style={{color:"#1890FF",border:"1px dashed #1890FF",padding:'3px 10px '}} htmlFor="ava">+点击上传图片</label>}

        <Button variant='contained' onClick={()=>handleSubmit()}>确定提交</Button>
        <CircularProgress style={{display:load}}/>
        </Stack>
        </div>
    )
}
export default Upload2