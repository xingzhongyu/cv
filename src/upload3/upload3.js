import { Stack, Button, CircularProgress } from "@mui/material";
import axios from "axios";
import { useState } from "react";

function Upload3(props) {
    const [color, setColor] = useState({});
    const [depth, setDepth] = useState({});
    const [color_show, set_color_show] = useState(false);
    const [depth_show, set_depth_show] = useState(false);

    const [color_src, set_color_src] = useState('');
    const [depth_src, set_depth_src] = useState('');
    const [load, set_load] = useState('none')

    const handle_color_change = (e) => {
        let file = e.target.files[0];
        e.preventDefault();
        let reader = new FileReader();
        reader.onloadend = () => {
            setColor(file);
            set_color_show(true);
            set_color_src(reader.result)
        }
        reader.readAsDataURL(file)
        // console.log(e);
    }

    const handle_depth_change = (e) => {
        let file = e.target.files[0];
        let reader = new FileReader();
        reader.onloadend = () => {
            setDepth(file);
            set_depth_show(true);
            set_depth_src(reader.result);
        }
        reader.readAsDataURL(file);
    }
    const handle_submit = (e) => {
        set_load('block')
        let formData = new FormData();
        formData.append('imgfile', color);
        formData.append('imgfile', depth);
        let config = {
            method: 'post',
            url: 'http://10.27.136.211:3000/img3',

            headers: {
                'Content-Type': 'multipart/form-data'
            },
            data: formData
        }
        axios(config).then(function (res) {
            let data = res.data;
            console.log(data)
           
            if (data.ans.body != undefined) {
                let ans = data.ans.body.split('\t');
                ans = ans.slice(1, ans.length);
                
                props.handle_points(ans);
            } else {
                props.handle_points([])
            }
            set_load('none');
        }).catch(function (err) {
            console.log(err);

        })


    }
    return (
        <div>
            <Stack spacing={2} direction="row">

                <Stack>
                    <p>上传彩色图片</p>
                    <input id="color" style={{ display: 'none' }} onChange={(e) => handle_color_change(e)} type="file"></input>
                    {color_show ? <label htmlFor="color" >

                        <img style={{ height: '80px', width: '80px' }} alt='' src={color_src} />
                    </label>
                        :
                        <label htmlFor="color" style={{ color: "#1890FF", border: "1px dashed #1890FF", padding: '3px 10px ' }}>+点击上传图片</label>}
                </Stack>
                <Stack>
                    <p>上传深度图片</p>
                    <input id="depth" style={{ display: 'none' }} onChange={(e) => handle_depth_change(e)} type="file"></input>
                    {depth_show ? <label htmlFor="depth" >
                        <img src={depth_src} style={{ height: '80px', width: '80px' }} alt='' />

                    </label> :
                        <label htmlFor="depth" style={{ color: "#1890FF", border: "1px dashed #1890FF", padding: '3px 10px ' }}>+点击上传图片</label>}
                </Stack>


            </Stack>
            <Button variant="contained" style={{ width: '74%', marginTop: '10px' }} onClick={(e) => handle_submit(e)}>提交</Button>
            <CircularProgress style={{ display: load }}></CircularProgress>
        </div>
    )

}
export default Upload3