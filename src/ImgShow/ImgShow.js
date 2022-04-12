import { useEffect, useRef } from "react"

function ImgShow(props){


    const canvas_ref=useRef(null);
    let colors=props.data;
    useEffect(()=>{
        canvas_ref.current.setAttribute('src',"data:image/png;base64,"+colors);

    })

    
    return(
        <div style={{border:'1px solid black'}}>
        <img id="myPic" ref={canvas_ref}></img>
        </div>
    )
}
export default ImgShow