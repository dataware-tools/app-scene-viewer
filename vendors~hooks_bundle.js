/*! For license information please see vendors~hooks_bundle.js.LICENSE.txt */
          background-color: hotpink;
          &:hover {
            color: ${"white"};
          }
        `},"div + emotion")),Sample=()=>import_react.default.createElement(SamplePresentation,null),import_KeyboardArrowDown=__toModule(require_KeyboardArrowDown()),import_KeyboardArrowUp=__toModule(require_KeyboardArrowUp()),import_react2=__toModule(__webpack_require__(0)),CurrentCaptionPresentation=({captions:propsCaptions,currentSceneIndex:currentSceneIndex,onChangeScene:onChangeScene})=>{const captions=[...propsCaptions],determinePlacement=itemIndex=>{const halfwayIndex=Math.ceil(captions.length/2);return currentSceneIndex===itemIndex?0:itemIndex>=halfwayIndex?currentSceneIndex>itemIndex-halfwayIndex?50*(itemIndex-currentSceneIndex):50*-(captions.length+currentSceneIndex-itemIndex):itemIndex>currentSceneIndex?50*(itemIndex-currentSceneIndex):itemIndex<currentSceneIndex?currentSceneIndex-itemIndex>=halfwayIndex?50*(captions.length-(currentSceneIndex-itemIndex)):50*-(currentSceneIndex-itemIndex):null};return import_react2.default.createElement(import_react2.default.Fragment,null,import_react2.default.createElement("div",{className:css`
          align-items: center;
          display: flex;
          flex-direction: row;
          overflow: auto;
        `},import_react2.default.createElement("div",{className:css`
            display: flex;
            flex-direction: column;
            flex-grow: 1;
            height: 150px;
            overflow-y: hidden;
            position: relative;
          `},captions.map(({timestamp:timestamp,caption:caption},index)=>{const isVisible=currentSceneIndex-1<=index&&index<=currentSceneIndex+1,isDisplayed=currentSceneIndex-2<=index&&index<=currentSceneIndex+2;return import_react2.default.createElement("div",{key:timestamp,className:css`
                  align-items: center;
                  bottom: 0;
                  color: ${index===currentSceneIndex?"white":"gray"};
                  display: flex;
                  display: ${isDisplayed?null:"none"};
                  flex-direction: row;
                  justify-content: center;
                  position: absolute;
                  top: 0;
                  transform: translateY(${determinePlacement(index)}px);
                  transition: transform 0.4s ease, opacity 0.4s ease;
                  visibility: ${isVisible?"visible":"hidden"};
                  width: 100%;
                `},import_react2.default.createElement("span",{className:css`
                    flex-shrink: 0;
                  `},timestamp),import_react2.default.createElement("span",{className:css`
                    flex-shrink: 0;
                    width: 10px;
                  `}),import_react2.default.createElement("div",{className:css`
                    overflow: hidden;
                    text-overflow: ${index!==currentSceneIndex&&"ellipsis"};
                    white-space: ${index!==currentSceneIndex&&"nowrap"};
                    word-break: ${index===currentSceneIndex&&"break-all"};
                  `},caption))})),import_react2.default.createElement("span",{className:css`
            flex-shrink: 0;
            width: 10px;
          `}),import_react2.default.createElement("div",{className:css`
            align-items: center;
            display: flex;
            flex-direction: column;
            flex-shrink: 0;
            justify-content: center;
            overflow: hidden;
            padding: 5px 0;
          `},import_react2.default.createElement("span",null,"前のシーン"),import_react2.default.createElement("span",{className:css`
              height: 5px;
            `}),import_react2.default.createElement("button",{onClick:()=>__async(void 0,null,(function*(){return yield onChangeScene(captions[currentSceneIndex-1])})),disabled:currentSceneIndex<1,className:css`
              align-items: center;
              border-radius: 100%;
              display: flex;
              height: 30px;
              justify-content: center;
              width: 30px;
            `},import_react2.default.createElement(import_KeyboardArrowUp.default,{fontSize:"large"})),import_react2.default.createElement("span",{className:css`
              height: 40px;
            `}),import_react2.default.createElement("button",{onClick:()=>__async(void 0,null,(function*(){return yield onChangeScene(captions[currentSceneIndex+1])})),disabled:currentSceneIndex>=captions.length-1,className:css`
              align-items: center;
              border-radius: 100%;
              display: flex;
              height: 30px;
              justify-content: center;
              width: 30px;
            `},import_react2.default.createElement(import_KeyboardArrowDown.default,{fontSize:"large"})),import_react2.default.createElement("span",{className:css`
              height: 5px;
            `}),import_react2.default.createElement("span",null,"次のシーン")),import_react2.default.createElement("span",{className:css`
            width: 10px;
          `})))},CurrentCaption=({captions:captions,currentTimestamp:currentTimestamp,onChangeScene:onChangeScene})=>{if(captions.length<=0)return import_react2.default.createElement("div",null,"No captions available");const currentSceneIndex=captions[captions.length-1].timestamp<=currentTimestamp?captions.length-1:captions.findIndex(({timestamp:timestamp},index)=>timestamp<=currentTimestamp&&currentTimestamp<captions[index+1].timestamp);return import_react2.default.createElement(CurrentCaptionPresentation,{captions:captions,currentSceneIndex:currentSceneIndex,onChangeScene:onChangeScene,currentTimestamp:currentTimestamp})},import_Search=__toModule(require_Search()),import_flexsearch=__toModule(require_flexsearch_bundle()),import_react4=__toModule(__webpack_require__(0)),import_react_highlight_words=__toModule(require_main()),import_react3=__toModule(__webpack_require__(0)),MarkerIcon=({color:color,fontColor:fontColor,size:size,text:text})=>import_react3.default.createElement("div",{className:css`
        background-color: ${color};
        border-radius: 50%;
        color: ${fontColor};
        height: ${size}px;
        line-height: ${size}px;
        text-align: center;
        width: ${size}px;
      `},text);MarkerIcon.defaultProps={color:"#3388FE",fontColor:"#FFFFFF",size:30,text:""};var SceneSelectorPresentation=({captionWithLabels:captionWithLabels,onSearch:onSearch,highlightedTexts:highlightedTexts,onSelectScene:onSelectScene})=>{const inputRef=(0,import_react4.useRef)(null),white="hsl(0, 0%, 94%)",gray="hsl(240, 2%, 53%)";return import_react4.default.createElement("div",{className:css`
        align-items: center;
        background-color: hsl(240deg 1% 41%);
        border-radius: 5px;
        color: white;
        display: flex;
        flex-direction: column;
        height: 100%;
        padding: 10px;
        width: 100%;
      `},import_react4.default.createElement("form",{onSubmit:e=>{var _a;e.preventDefault(),onSearch((null==(_a=inputRef.current)?void 0:_a.value)||"")},className:css`
          display: flex;
          flex-direction: row;
          flex-shrink: 0;
          position: relative;
        `},import_react4.default.createElement(import_Search.default,{fontSize:"large",className:css`
            color: ${gray};
            left: 12px;
            position: absolute;
            top: 4px;
          `}),import_react4.default.createElement("input",{ref:inputRef,className:css`
            background-color: ${white};
            color: ${gray};
            font-size: 1.25rem;
            padding-left: 40px;
            &:focus {
              background-color: ${white};
              color: ${gray};
            }
          `}),import_react4.default.createElement("span",{className:css`
            width: 10px;
          `}),import_react4.default.createElement("button",{onClick:()=>{var _a;return onSearch((null==(_a=inputRef.current)?void 0:_a.value)||"")},className:css`
            background-color: ${white};
            color: black;
            font-size: 1.25rem;
            font-weight: bold;
            padding: 0 20px;
            &:hover {
              background-color: ${white} !important;
              color: ${gray} !important;
            }
          `},"Search")),import_react4.default.createElement("span",{className:css`
          flex-shrink: 0;
          height: 10px;
        `}),import_react4.default.createElement("div",{className:css`
          flex-shrink: 1;
          height: 100%;
          overflow: auto;
          padding: 5px 10px;
          width: 100%;
        `},import_react4.default.createElement("ul",{className:css`
            display: inline-block;
          `},captionWithLabels.map(({timestamp:timestamp,caption:caption,label:label})=>import_react4.default.createElement("li",{key:timestamp,className:css`
                align-items: center;
                cursor: pointer;
                display: flex;
                flex-direction: row;
                padding: 5px;
                &:hover {
                  background-color: ${"hsl(240, 2%, 70%)"};
                }
              `,onClick:()=>onSelectScene(timestamp)},import_react4.default.createElement("span",{className:css`
                  align-items: center;
                  height: 20px;
                  justify-content: center;
                  width: 20px;
                `},label&&import_react4.default.createElement(MarkerIcon,{size:20,text:label})),import_react4.default.createElement("span",{className:css`
                  flex-shrink: 0;
                  width: 10px;
                `}),import_react4.default.createElement("span",{className:css`
                  font-weight: bold;
                `},timestamp),import_react4.default.createElement("span",{className:css`
                  flex-shrink: 0;
                  width: 10px;
                `}),import_react4.default.createElement("span",{className:css`
                  flex-grow: 1;
                  white-space: nowrap;
                `},import_react4.default.createElement(import_react_highlight_words.default,{textToHighlight:caption,searchWords:highlightedTexts,highlightStyle:{backgroundColor:"hsl(197, 39%, 58%)",fontWeight:"bold",color:"white"}})))))))},SceneSelector=_a=>{var _b=_a,{captions:captions,setPinLocations:setPinLocations}=_b,delegated=__objRest(_b,["captions","setPinLocations"]);const[highlightedTexts,setHighlightedTexts]=(0,import_react4.useState)([""]),[captionWithLabels,setCaptionWithLabels]=(0,import_react4.useState)([]),flexSearchRef=(0,import_react4.useRef)(new import_flexsearch.Index({tokenize:"full"})),setCaptions=captions2=>{const captionWithLabels2=(captions2=>{let numNonLabeledCaption=0;return captions2.map((caption,index)=>{const label=caption.location?(index+1-numNonLabeledCaption).toString():void 0;return caption.location||(numNonLabeledCaption+=1),__spreadProps(__spreadValues({},caption),{label:label})})})(captions2);setCaptionWithLabels(captionWithLabels2),setPinLocations((captionWithLabels2=>captionWithLabels2.filter(({label:label})=>label).map(({timestamp:timestamp,caption:caption,location:location,label:label})=>({label:label,longitude:null==location?void 0:location.longitude,latitude:null==location?void 0:location.latitude,altitude:null==location?void 0:location.altitude,description:caption,timestamp:timestamp})))(captionWithLabels2))};(0,import_react4.useEffect)(()=>{captions.forEach(({caption:caption},index)=>flexSearchRef.current.add(index,caption)),setCaptions(captions)},[captions]);return import_react4.default.createElement(SceneSelectorPresentation,__spreadValues({captionWithLabels:captionWithLabels,highlightedTexts:highlightedTexts,onSearch:searchText=>{const fixedSearchText=searchText.replace("　"," ");if(setHighlightedTexts(fixedSearchText.split(" ")),!fixedSearchText)return void setCaptions(captions);const searchResults=new Set(flexSearchRef.current.search(fixedSearchText)),newCaptions=captions.filter((_,index)=>searchResults.has(index));setCaptions(newCaptions)}},delegated))},import_GpsFixed=__toModule(require_GpsFixed()),import_leaflet11=__toModule(require_leaflet_src()),import_react17=__toModule(__webpack_require__(0)),import_server2=__toModule(require_server_browser()),import_react5=__toModule(__webpack_require__(0));function useAttribution(map,attribution){const attributionRef=(0,import_react5.useRef)(attribution);(0,import_react5.useEffect)((function(){attribution!==attributionRef.current&&null!=map.attributionControl&&(null!=attributionRef.current&&map.attributionControl.removeAttribution(attributionRef.current),null!=attribution&&map.attributionControl.addAttribution(attribution)),attributionRef.current=attribution}),[map,attribution])}function updateCircle(layer,props,prevProps){props.center!==prevProps.center&&layer.setLatLng(props.center),null!=props.radius&&props.radius!==prevProps.radius&&layer.setRadius(props.radius)}var import_react7=__toModule(__webpack_require__(0)),import_react_dom=__toModule(require_react_dom()),import_react6=__toModule(__webpack_require__(0)),CONTEXT_VERSION=1,LeafletContext=(0,import_react6.createContext)(null),LeafletProvider=LeafletContext.Provider;function useLeafletContext(){const context=(0,import_react6.useContext)(LeafletContext);if(null==context)throw new Error("No context provided: useLeafletContext() can only be used in a descendant of <MapContainer>");return context}function createContainerComponent(useElement){function ContainerComponent(props,ref){const{instance:instance,context:context}=useElement(props).current;return(0,import_react7.useImperativeHandle)(ref,()=>instance),null==props.children?null:import_react7.default.createElement(LeafletProvider,{value:context},props.children)}return(0,import_react7.forwardRef)(ContainerComponent)}function createDivOverlayComponent(useElement){function OverlayComponent(props,ref){const[isOpen,setOpen]=(0,import_react7.useState)(!1),{instance:instance}=useElement(props,setOpen).current;(0,import_react7.useImperativeHandle)(ref,()=>instance),(0,import_react7.useEffect)((function(){isOpen&&instance.update()}),[instance,isOpen,props.children]);const contentNode=instance._contentNode;return contentNode?(0,import_react_dom.createPortal)(props.children,contentNode):null}return(0,import_react7.forwardRef)(OverlayComponent)}function createLeafComponent(useElement){function LeafComponent(props,ref){const{instance:instance}=useElement(props).current;return(0,import_react7.useImperativeHandle)(ref,()=>instance),null}return(0,import_react7.forwardRef)(LeafComponent)}var import_react8=__toModule(__webpack_require__(0));function useEventHandlers(element,eventHandlers){const eventHandlersRef=(0,import_react8.useRef)();(0,import_react8.useEffect)((function(){return null!=eventHandlers&&element.instance.on(eventHandlers),eventHandlersRef.current=eventHandlers,function(){null!=eventHandlersRef.current&&element.instance.off(eventHandlersRef.current),eventHandlersRef.current=null}}),[element,eventHandlers])}function withPane(props,context){var _props$pane;const pane=null!=(_props$pane=props.pane)?_props$pane:context.pane;return pane?__spreadProps(__spreadValues({},props),{pane:pane}):props}function createDivOverlayHook(useElement,useLifecycle){return function(props,setOpen){const context=useLeafletContext(),elementRef=useElement(withPane(props,context),context);return useAttribution(context.map,props.attribution),useEventHandlers(elementRef.current,props.eventHandlers),useLifecycle(elementRef.current,context,props,setOpen),elementRef}}var import_leaflet=__toModule(require_leaflet_src());function splitClassName(className){return className.split(" ").filter(Boolean)}function addClassName(element,className){splitClassName(className).forEach(cls=>{import_leaflet.DomUtil.addClass(element,cls)})}var import_react9=__toModule(__webpack_require__(0));function createElementHook(createElement,updateElement){return null==updateElement?function(props,context){return(0,import_react9.useRef)(createElement(props,context))}:function(props,context){const elementRef=(0,import_react9.useRef)(createElement(props,context)),propsRef=(0,import_react9.useRef)(props),{instance:instance}=elementRef.current;return(0,import_react9.useEffect)((function(){propsRef.current!==props&&(updateElement(instance,props,propsRef.current),propsRef.current=props)}),[instance,props,context]),elementRef}}var import_react10=__toModule(__webpack_require__(0));function useLayerLifecycle(element,context){(0,import_react10.useEffect)((function(){var _context$layerContain;const container=null!=(_context$layerContain=context.layerContainer)?_context$layerContain:context.map;return container.addLayer(element.instance),function(){container.removeLayer(element.instance)}}),[context,element])}function createLayerHook(useElement){return function(props){const context=useLeafletContext(),elementRef=useElement(withPane(props,context),context);return useAttribution(context.map,props.attribution),useEventHandlers(elementRef.current,props.eventHandlers),useLayerLifecycle(elementRef.current,context),elementRef}}var import_react11=__toModule(__webpack_require__(0));function usePathOptions(element,props){const optionsRef=(0,import_react11.useRef)();(0,import_react11.useEffect)((function(){if(props.pathOptions!==optionsRef.current){var _props$pathOptions;const options=null!=(_props$pathOptions=props.pathOptions)?_props$pathOptions:{};element.instance.setStyle(options),optionsRef.current=options}}),[element,props])}function createPathHook(useElement){return function(props){const context=useLeafletContext(),elementRef=useElement(withPane(props,context),context);return useEventHandlers(elementRef.current,props.eventHandlers),useLayerLifecycle(elementRef.current,context),usePathOptions(elementRef.current,props),elementRef}}function createLayerComponent(createElement,updateElement){return createContainerComponent(createLayerHook(createElementHook(createElement,updateElement)))}function createOverlayComponent(createElement,useLifecycle){return createDivOverlayComponent(createDivOverlayHook(createElementHook(createElement),useLifecycle))}function createPathComponent(createElement,updateElement){return createContainerComponent(createPathHook(createElementHook(createElement,updateElement)))}function createTileLayerComponent(createElement,updateElement){return createLeafComponent(createLayerHook(createElementHook(createElement,updateElement)))}function updateGridLayer(layer,props,prevProps){const{opacity:opacity,zIndex:zIndex}=props;null!=opacity&&opacity!==prevProps.opacity&&layer.setOpacity(opacity),null!=zIndex&&zIndex!==prevProps.zIndex&&layer.setZIndex(zIndex)}var import_leaflet2=__toModule(require_leaflet_src()),CircleMarker=createPathComponent((function(_a,ctx){var _b=_a,{center:center,children:_c}=_b,options=__objRest(_b,["center","children"]);const instance=new import_leaflet2.CircleMarker(center,options);return{instance:instance,context:__spreadProps(__spreadValues({},ctx),{overlayContainer:instance})}}),updateCircle),import_leaflet3=__toModule(require_leaflet_src()),import_react12=__toModule(__webpack_require__(0));function _extends(){return(_extends=Object.assign||function(target){for(var i=1;i<arguments.length;i++){var source=arguments[i];for(var key in source)Object.prototype.hasOwnProperty.call(source,key)&&(target[key]=source[key])}return target}).apply(this,arguments)}function useMapElement(mapRef,props){const[map,setMap]=(0,import_react12.useState)(null);return(0,import_react12.useEffect)(()=>{if(null!==mapRef.current&&null===map){const instance=new import_leaflet3.Map(mapRef.current,props);null!=props.center&&null!=props.zoom?instance.setView(props.center,props.zoom):null!=props.bounds&&instance.fitBounds(props.bounds,props.boundsOptions),null!=props.whenReady&&instance.whenReady(props.whenReady),setMap(instance)}},[mapRef,map,props]),map}function MapContainer(_a){var _b=_a,{children:children,className:className,id:id,placeholder:placeholder,style:style,whenCreated:whenCreated}=_b,options=__objRest(_b,["children","className","id","placeholder","style","whenCreated"]);const mapRef=(0,import_react12.useRef)(null),map=useMapElement(mapRef,options),createdRef=(0,import_react12.useRef)(!1);(0,import_react12.useEffect)(()=>{null!=map&&!1===createdRef.current&&null!=whenCreated&&(createdRef.current=!0,whenCreated(map))},[map,whenCreated]);const[props]=(0,import_react12.useState)({className:className,id:id,style:style}),context=(0,import_react12.useMemo)(()=>map?{__version:CONTEXT_VERSION,map:map}:null,[map]),contents=context?import_react12.default.createElement(LeafletProvider,{value:context},children):null!=placeholder?placeholder:null;return import_react12.default.createElement("div",_extends({},props,{ref:mapRef}),contents)}var import_leaflet4=__toModule(require_leaflet_src()),Marker=createLayerComponent((function(_a,ctx){var _b=_a,{position:position2}=_b,options=__objRest(_b,["position"]);const instance=new import_leaflet4.Marker(position2,options);return{instance:instance,context:__spreadProps(__spreadValues({},ctx),{overlayContainer:instance})}}),(function(marker,props,prevProps){props.position!==prevProps.position&&marker.setLatLng(props.position),null!=props.icon&&props.icon!==prevProps.icon&&marker.setIcon(props.icon),null!=props.zIndexOffset&&props.zIndexOffset!==prevProps.zIndexOffset&&marker.setZIndexOffset(props.zIndexOffset),null!=props.opacity&&props.opacity!==prevProps.opacity&&marker.setOpacity(props.opacity),null!=marker.dragging&&props.draggable!==prevProps.draggable&&(!0===props.draggable?marker.dragging.enable():marker.dragging.disable())})),import_react13=__toModule(__webpack_require__(0)),import_react_dom2=__toModule(require_react_dom()),DEFAULT_PANES=["mapPane","markerPane","overlayPane","popupPane","shadowPane","tilePane","tooltipPane"];function omitPane(obj,pane){const _a=obj,{[pane]:_p}=_a;return __objRest(_a,[__restKey(pane)])}function createPane(props,context){var _props$pane;const name=props.name;if(-1!==DEFAULT_PANES.indexOf(name))throw new Error("You must use a unique name for a pane that is not a default Leaflet pane: "+name);if(null!=context.map.getPane(name))throw new Error("A pane with this name already exists: "+name);const parentPaneName=null!=(_props$pane=props.pane)?_props$pane:context.pane,parentPane=parentPaneName?context.map.getPane(parentPaneName):void 0,element=context.map.createPane(name,parentPane);return null!=props.className&&addClassName(element,props.className),null!=props.style&&Object.keys(props.style).forEach(key=>{element.style[key]=props.style[key]}),element}function Pane(props){const[paneElement,setPaneElement]=(0,import_react13.useState)(),context=useLeafletContext(),newContext=(0,import_react13.useMemo)(()=>__spreadProps(__spreadValues({},context),{pane:props.name}),[context]);return(0,import_react13.useEffect)(()=>(setPaneElement(createPane(props,context)),function(){const pane=context.map.getPane(props.name);null==pane||null==pane.remove||pane.remove(),null!=context.map._panes&&(context.map._panes=omitPane(context.map._panes,props.name),context.map._paneRenderers=omitPane(context.map._paneRenderers,props.name))}),[]),null!=props.children&&null!=paneElement?(0,import_react_dom2.createPortal)(import_react13.default.createElement(LeafletProvider,{value:newContext},props.children),paneElement):null}var import_leaflet5=__toModule(require_leaflet_src()),Polyline=createPathComponent((function(_a,ctx){var _b=_a,{positions:positions}=_b,options=__objRest(_b,["positions"]);const instance=new import_leaflet5.Polyline(positions,options);return{instance:instance,context:__spreadProps(__spreadValues({},ctx),{overlayContainer:instance})}}),(function(layer,props,prevProps){props.positions!==prevProps.positions&&layer.setLatLngs(props.positions)})),import_leaflet6=__toModule(require_leaflet_src()),import_react14=__toModule(__webpack_require__(0)),Popup=createOverlayComponent((function(props,context){return{instance:new import_leaflet6.Popup(props,context.overlayContainer),context:context}}),(function(element,context,props,setOpen){const{onClose:onClose,onOpen:onOpen,position:position2}=props;(0,import_react14.useEffect)((function(){const{instance:instance}=element;function onPopupOpen(event){event.popup===instance&&(instance.update(),setOpen(!0),null==onOpen||onOpen())}function onPopupClose(event){event.popup===instance&&(setOpen(!1),null==onClose||onClose())}return context.map.on({popupopen:onPopupOpen,popupclose:onPopupClose}),null==context.overlayContainer?(null!=position2&&instance.setLatLng(position2),instance.openOn(context.map)):context.overlayContainer.bindPopup(instance),function(){context.map.off({popupopen:onPopupOpen,popupclose:onPopupClose}),null==context.overlayContainer?context.map.removeLayer(instance):context.overlayContainer.unbindPopup()}}),[element,context,setOpen,onClose,onOpen,position2])})),import_leaflet7=__toModule(require_leaflet_src()),TileLayer=createTileLayerComponent((function(_a,context){var _b=_a,{url:url}=_b,options=__objRest(_b,["url"]);return{instance:new import_leaflet7.TileLayer(url,withPane(options,context)),context:context}}),updateGridLayer),import_react15=__toModule(__webpack_require__(0)),CurrentLocationMarker=({position:position2})=>import_react15.default.createElement(CircleMarker,{center:position2,pathOptions:{color:"#E3EEFF",fillColor:"#3388FE",fillOpacity:1,weight:5},radius:10}),import_leaflet9=__toModule(require_leaflet_src()),import_react16=__toModule(__webpack_require__(0)),import_server=__toModule(require_server_browser()),MarkerWithText=({longitude:longitude,latitude:latitude,popupText:popupText,size:size,text:text})=>{const positionShift=(0,import_react16.useMemo)(()=>size?size/2-3:0,[size]),icon=(0,import_leaflet9.divIcon)({html:`\n      <div style="\n        position: relative;\n        top: -${positionShift}px;\n        left: -${positionShift}px;\n      ">\n        ${import_server.default.renderToString(import_react16.default.createElement(MarkerIcon,{text:text,size:size}))}\n      </div>\n    `,popupAnchor:[0,-positionShift-5]});return import_react16.default.createElement(Marker,{position:[latitude,longitude],icon:icon},popupText&&import_react16.default.createElement(Popup,null,popupText))};MarkerWithText.defaultProps={size:30},import_leaflet11.default.Icon.Default.imagePath="//cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/";var mapInitialZoom=15,MapPanelPresentation=({centerPosition:centerPosition,currentPosition:currentPosition,markers:markers,polylines:polylines,setMap:setMap})=>import_react17.default.createElement(MapContainer,{center:centerPosition,zoom:mapInitialZoom,scrollWheelZoom:!1,whenCreated:setMap,className:css`
        height: 100%;
        width: 100%;
      `},import_react17.default.createElement(TileLayer,{attribution:'© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',url:"https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"}),import_react17.default.createElement(Pane,{name:"polylines",style:{zIndex:800}},polylines&&polylines.map((polyline,i)=>import_react17.default.createElement(Polyline,{positions:polyline.positions,key:i}))),import_react17.default.createElement(Pane,{name:"markers",style:{zIndex:900}},markers&&markers.map((marker,i)=>import_react17.default.createElement(MarkerWithText,{key:i,longitude:marker.longitude,latitude:marker.latitude,popupText:marker.popupText,size:30,text:marker.text}))),import_react17.default.createElement(Pane,{name:"current-position",style:{zIndex:1e3}},currentPosition&&import_react17.default.createElement(CurrentLocationMarker,{position:currentPosition}))),MapPanel=({centerPosition:centerPosition,currentPosition:currentPosition,markers:markers,polylines:polylines})=>{const[map,setMap]=(0,import_react17.useState)(void 0),[trackCurrentPosition,setTrackCurrentPosition]=(0,import_react17.useState)(!1);(0,import_react17.useEffect)(()=>{map&&currentPosition&&trackCurrentPosition&&map.setView(currentPosition,mapInitialZoom)},[map,currentPosition,trackCurrentPosition]);const onDragStart=(0,import_react17.useCallback)(()=>{setTrackCurrentPosition(!1)},[]);return(0,import_react17.useEffect)(()=>{if(map)return map.on("dragstart",onDragStart),()=>{map.off("dragstart",onDragStart)}},[map,onDragStart]),(0,import_react17.useEffect)(()=>{if(!map)return;const trackCurrentPositionButton=import_leaflet11.default.DomUtil.create("div","leaflet-bar");trackCurrentPositionButton.innerHTML=`\n      ${import_server2.default.renderToString(import_react17.default.createElement("a",{role:"button",className:css`
            cursor: pointer;
            font-size: 22px;
//# sourceMappingURL=vendors~hooks_bundle.js.map