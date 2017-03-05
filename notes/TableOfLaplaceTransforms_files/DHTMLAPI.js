// DHTML3API.js custom API for cross-platform
// object positioning by Danny Goodman (http://www.dannyg.com).
// Release 3.0. Supports NN4, IE, and W3C DOMs.
// JavaScript & DHTML Cookbook, 2nd Edition 13.3


// Found this really nice browser detect at 
// http://www.quirksmode.org/blog/archives/2006/07/browser_detect.html

var BrowserDetect = {
	init: function () {
		this.browser = this.searchString(this.dataBrowser) || "An unknown browser";
		this.version = this.searchVersion(navigator.userAgent)
			|| this.searchVersion(navigator.appVersion)
			|| "an unknown version";
		this.OS = this.searchString(this.dataOS) || "an unknown OS";
	},
	searchString: function (data) {
		for (var i=0;i<data.length;i++)	{
			var dataString = data[i].string;
			var dataProp = data[i].prop;
			this.versionSearchString = data[i].versionSearch || data[i].identity;
			if (dataString) {
				if (dataString.indexOf(data[i].subString) != -1)
					return data[i].identity;
			}
			else if (dataProp)
				return data[i].identity;
		}
	},
	searchVersion: function (dataString) {
		var index = dataString.indexOf(this.versionSearchString);
		if (index == -1) return;
		return parseFloat(dataString.substring(index+this.versionSearchString.length+1));
	},
	dataBrowser: [
		{ 	string: navigator.userAgent,
			subString: "OmniWeb",
			versionSearch: "OmniWeb/",
			identity: "OmniWeb"
		},
		{
			string: navigator.vendor,
			subString: "Apple",
			identity: "Safari"
		},
		{
			prop: window.opera,
			identity: "Opera"
		},
		{
			string: navigator.vendor,
			subString: "iCab",
			identity: "iCab"
		},
		{
			string: navigator.vendor,
			subString: "KDE",
			identity: "Konqueror"
		},
		{
			string: navigator.userAgent,
			subString: "Firefox",
			identity: "Firefox"
		},
		{
			string: navigator.vendor,
			subString: "Camino",
			identity: "Camino"
		},
		{		// for newer Netscapes (6+)
			string: navigator.userAgent,
			subString: "Netscape",
			identity: "Netscape"
		},
		{
			string: navigator.userAgent,
			subString: "MSIE",
			identity: "Explorer",
			versionSearch: "MSIE"
		},
		{
			string: navigator.userAgent,
			subString: "Gecko",
			identity: "Mozilla",
			versionSearch: "rv"
		},
		{ 		// for older Netscapes (4-)
			string: navigator.userAgent,
			subString: "Mozilla",
			identity: "Netscape",
			versionSearch: "Mozilla"
		}
	],
	dataOS : [
		{
			string: navigator.platform,
			subString: "Win",
			identity: "Windows"
		},
		{
			string: navigator.platform,
			subString: "Mac",
			identity: "Mac"
		},
		{
			string: navigator.platform,
			subString: "Linux",
			identity: "Linux"
		}
	]

};
BrowserDetect.init();



var DHTMLAPI = {
    browserClass : new Object(),
    init : function () {
        this.browserClass.isCSS = ((document.body && document.body.style) ? true : false);
        this.browserClass.isW3C = ((this.browserClass.isCSS && document.getElementById) ?
            true : false),
        this.browserClass.isIE4 = ((this.browserClass.isCSS && document.all) ?
            true : false),
        this.browserClass.isNN4 = ((document.layers) ? true : false),
        this.browserClass.isIECSSCompat = ((document.compatMode &&
            document.compatMode.indexOf("CSS1") >= 0) ? true : false)
    },
    // Seek nested NN4 layer from string name
    seekLayer : function (doc, name) {
        var elem;
        for (var i = 0; i < doc.layers.length; i++) {
            if (doc.layers[i].name == name) {
                elem = doc.layers[i];
                break;
            }
            // dive into nested layers if necessary
            if (doc.layers[i].document.layers.length > 0) {
                elem = this.seekLayer(doc.layers[i].document, name);
                if (elem) {break;}
            }
        }
        return elem;
    },

    // Convert element name string or object reference
    // into a valid element object reference
    getRawObject : function (elemRef) {
        var elem;
        if (typeof elemRef == "string") {
            if (this.browserClass.isW3C) {
                elem = document.getElementById(elemRef);
            } else if (this.browserClass.isIE4) {
                 elem = document.all(elemRef);
            } else if (this.browserClass.isNN4) {
                elem = this.seekLayer(document, elemRef);
            }
        } else {
            // pass through object reference
            elem = elemRef;
        }
        return elem;
    },
    // Convert element name string or object reference
    // into a valid style (or NN4 layer) object reference
    getStyleObject : function (elemRef) {
        var elem = this.getRawObject(elemRef);
        if (elem && this.browserClass.isCSS) {
            elem = elem.style;
        }
        return elem;
    },
    // Position an element at a specific pixel coordinate
    moveTo : function (elemRef, x, y) {
        var elem = this.getStyleObject(elemRef);
        if (elem) {
            if (this.browserClass.isCSS) {
                // equalize incorrect numeric value type
                var units = (typeof elem.left == "string") ? "px" : 0;
                elem.left = x + units;
                elem.top = y + units;
            } else if (this.browserClass.isNN4) {
                elem.moveTo(x,y);
            }
        }
    },

    // Move an element by x and/or y pixels
    moveBy : function (elemRef, deltaX, deltaY) {
        var elem = this.getStyleObject(elemRef);
        if (elem) {
            if (this.browserClass.isCSS) {
                // equalize incorrect numeric value type
                var units = (typeof elem.left == "string") ? "px" : 0;
                if (!isNaN(this.getElementLeft(elemRef))) {
                    elem.left = this.getElementLeft(elemRef) + deltaX + units;
                    elem.top = this.getElementTop(elemRef) + deltaY + units;
                }
            } else if (this.browserClass.isNN4) {
                elem.moveBy(deltaX, deltaY);
            }
        }
    },

    // Set the z-order of an object
    setZIndex : function (obj, zOrder) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            elem.zIndex = zOrder;
        }
    },
    
    // Set the display of an object
    // Added by Paul Dawkins
    setDisplay : function (obj, disp) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            if (this.browserClass.isCSS) {
                elem.display = disp;
            }
        }
    },

    // Set the background color of an object
    setBGColor : function (obj, color) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            if (this.browserClass.isCSS) {
                elem.backgroundColor = color;
            } else if (this.browserClass.isNN4) {
                elem.bgColor = color;
            }
        }
    },
    
    // Set the background image of an object
    // Added by Paul Dawkins
    setBGImage : function (obj, img, repeat, position) {
        var elem = this.getStyleObject(obj);
        if (elem && img != null) {
            if (this.browserClass.isCSS) {
                elem.backgroundImage = "url(" + img + ")";
                
                if (repeat != null && repeat != "") {
                    elem.backgroundRepeat = repeat;
                }
                if (position != null && position != "") {
                    elem.backgroundPosition = position;
                }
            }
        }
    },
    
    // Set the border width of an object
    // Added by Paul Dawkins
    setBorderWidth : function (obj, top, bot, right, left) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            if (this.browserClass.isCSS) {
                if (top != null && top != "") {
                    elem.borderTopWidth = top;
                }
                if (bot != null && bot != "") {
                    elem.borderBottomWidth = bot;
                }
                if (right != null && right != "") {
                    elem.borderRightWidth = right;
                }
                if (left != null && left != "") {
                    elem.borderLeftWidth = left;
                }
            }
        }
    },
    
    // Set the width of an object
    // Added by Paul Dawkins
    setWidth : function (obj, width) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            if (this.browserClass.isCSS) {
                elem.width = width;
            }
        }
    },
    
    // Set the height of an object
    // Added by Paul Dawkins
    setHeight : function (obj, height) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            if (this.browserClass.isCSS) {
                elem.height = height;
            }
        }
    },
    
    // Set the visibility of an object to visible
    show : function (obj) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            elem.visibility = "visible";
        }
    },

    // Set the visibility of an object to hidden
    hide : function (obj) {
        var elem = this.getStyleObject(obj);
        if (elem) {
            elem.visibility = "hidden";
        }
    },

    // return computed value for an element's style property
    getComputedStyle : function (elemRef, CSSStyleProp) {
        var elem = this.getRawObject(elemRef);
        var styleValue, camel;
        if (elem) {
            if (document.defaultView && document.defaultView.getComputedStyle) {
                // W3C DOM version
                var compStyle = document.defaultView.getComputedStyle(elem, "");
                styleValue = compStyle.getPropertyValue(CSSStyleProp);
            } else if (elem.currentStyle) {
                // make IE style property camelCase name from CSS version
                var IEStyleProp = CSSStyleProp;
                var re = /-\D/;
                while (re.test(IEStyleProp)) {
                    camel = IEStyleProp.match(re)[0].charAt(1).toUpperCase();
                   IEStyleProp = IEStyleProp.replace(re, camel);
                }
                styleValue = elem.currentStyle[IEStyleProp];
            }
        }
        return (styleValue) ? styleValue : null;
    },

    // Retrieve the x coordinate of a positionable object
    getElementLeft : function (elemRef) {
        var elem = this.getRawObject(elemRef);
        var result = null;
        if (this.browserClass.isCSS || this.browserClass.isW3C) {
            result = parseInt(this.getComputedStyle(elem, "left"));
        } else if (this.browserClass.isNN4) {
            result = elem.left;
        }
        return result;
    },

    // Retrieve the y coordinate of a positionable object
    getElementTop : function (elemRef) {
        var elem = this.getRawObject(elemRef);
        var result = null;
        if (this.browserClass.isCSS || this.browserClass.isW3C) {
            result = parseInt(this.getComputedStyle(elem, "top"));
        } else if (this.browserClass.isNN4) {
            result = elem.top;
        }
        return result;
    },

    // Retrieve the rendered width of an element
    getElementWidth : function (elemRef) {
        var result = null;
        var elem = this.getRawObject(elemRef);
        if (elem) {
            if (elem.offsetWidth) {
                if (elem.scrollWidth && (elem.offsetWidth != elem.scrollWidth)) {
                    result = elem.scrollWidth;
                } else {
                    result = elem.offsetWidth;
                }
            } else if (elem.clip && elem.clip.width) {
                // Netscape 4 positioned elements
                result = elem.clip.width;
            }
        }
        return result;
    },

    // Retrieve the rendered height of an element
    getElementHeight : function (elemRef) {
        var result = null;
        var elem = this.getRawObject(elemRef);
        if (elem) {
            if (elem.offsetHeight) {
                result = elem.offsetHeight;
            } else if (elem.clip && elem.clip.height) {
                result = elem.clip.height;
            }
        }
        return result;
    },

    // Return the available content width space in browser window
    getInsideWindowWidth : function () {
        if (window.innerWidth) {
            return window.innerWidth;
        } else if (this.browserClass.isIECSSCompat) {
            // measure the html element's clientWidth
            return document.body.parentElement.clientWidth;
        } else if (document.body && document.body.clientWidth) {
            return document.body.clientWidth;
        }
        return null;
    },
 
    // Return the available content height space in browser window
    getInsideWindowHeight : function () {
        if (window.innerHeight) {
            return window.innerHeight;
        } else {
            if (document.body.clientHeight != document.body.parentNode.clientHeight) {
                // measure the html element's clientHeight
                // I commented this out because it kept returning zero in IE7 - Paul Dawkins...
                //return document.body.parentNode.clientHeight;
                return document.body.clientHeight;
            } else {
                return document.body.clientHeight;
            }
        }
        return null;
    },
    
    // Added by Paul Dawkins
    getPageEventCoords : function (evt) {
	    var coords = {left:0, top:0};
	    if (evt.pageX) {
	        coords.left = evt.pageX;
	        coords.top = evt.pageY;
	    } else if (evt.clientX){
	        coords.left =
	            evt.clientX + document.body.scrollLeft - document.body.clientLeft;
	        coords.top =
	            evt.clientY + document.body.scrollTop - document.body.clientTop;
	        // include html element space, if applicable
	        if (document.body.parentElement && document.body.parentElement.clientLeft) {
	            var bodParent = document.body.parentElement;
	            coords.left += bodParent.scrollLeft - bodParent.clientLeft;
	            coords.top += bodParent.scrollTop - bodParent.clientTop;
	        }
	    }
	    return coords;
	},
	
	// Added by Paul Dawkins with some modifications...
    // I had to modify the evt.layerX part to get it to work with Firefox....
    // cancelevt - cancel the event?
    getPositionedEventCoords: function (evt, cancelevt) {

        if (arguments.length == 1) {
            cancelevt = true;
        }
	    var elem = (evt.target) ? evt.target : evt.srcElement;
	    var coords = {left:0, top:0};
	    
	    if (evt.layerX) {
			var elemRef = (elem.hasAttributes() && elem.getAttribute("ID")!= null) ? elem.getAttribute("ID") : ""
			var borders = this.getElementPosition(elemRef)
	        
	        coords.left = evt.layerX - borders.left;
	        coords.top = evt.layerY - borders.top;
	    } else if (evt.offsetX) {
	        coords.left = evt.offsetX;
	        coords.top = evt.offsetY;
	    }

	    if (cancelevt) {
	        evt.cancelBubble = true;
	    }

	    return coords;
	},
	
	// Added by Paul Dawkins
	// This returns the the left and top absolute coordinates of an element
	getElementPosition : function (elemID) {
	    var offsetTrail = document.getElementById(elemID);
	    var offsetLeft = 0;
	    var offsetTop = 0;
	    while (offsetTrail) {
	        offsetLeft += offsetTrail.offsetLeft;
	        offsetTop += offsetTrail.offsetTop;
	        offsetTrail = offsetTrail.offsetParent;
	    }
	    return {left:offsetLeft, top:offsetTop};
	},
	
	// Added by Paul Dawkins some heavy modification for firefox....
	adjustIFrameSize : function (elemID) {
	    var myIframe = document.getElementById(elemID);
	    
	    if (myIframe) {
	        if (myIframe.contentDocument && myIframe.contentDocument.body.scrollHeight) {
	            // W3C DOM iframe document syntax
	            // Firefox (and if we have firefox we'll be here...) won't 
	            // shrink the IFrame so set this to something small first and
	            // then set the height to the scroll height and it will show
	            // up correctly....
	            //
	            // however it causes a flicker and plays havoc with scroll position.
	            //myIframe.height = 20;
	            myIframe.height = myIframe.contentDocument.body.scrollHeight;
	        } else if (myIframe.Document && myIframe.Document.body.scrollHeight) {
	            // IE DOM syntax
	            myIframe.height = myIframe.Document.body.scrollHeight;
	        }
	    }
	},
	
	// Added by Paul Dawkins with some heavy modification....
	// This is the event handler that should be tied to the onload event of 
	// an iframe....
	resizeIframe : function (evt) {
	    evt = (evt) ? evt : event;
	    
	    // This was from the original and simply didn't work....
	    //var target = (evt.target) ? evt.target : evt.srcElement;
	    
	    // This covers all the possibilities that I could think of....
	    if (evt.target) {
			target = evt.target;
	    } else if (evt.currentTarget) {
			target = evt.currentTarget;
	    } else if (evt.srcElement) {
			target = evt.srcElement;
	    } else {
			target = null;
	    }
	    
	    //alert(target.tagName);
	    //alert(target.id);
	    
	    // take care of W3C event processing from iframe's root document
	    //if (target.nodeType == 9) {
	    //  if (evt.currentTarget &&
	    //      evt.currentTarget.tagName.toLowerCase() == "iframe") {
	    //        target = evt.currentTarget;
	    //    }
	    //}
	    if (target) {
	        DHTMLAPI.adjustIFrameSize(target.id);
	    }
	}



}

addOnLoadEvent(function() {DHTMLAPI.init()});
