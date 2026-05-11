// // Show warning banner when displaying old content versions
// window.addEventListener("DOMContentLoaded", function() {
//     var xhr = new XMLHttpRequest();
//     xhr.open("GET", "/versions.json");
//     xhr.onload = function() {
//         var versions = JSON.parse(this.responseText);
//         latest_version = ""
//         for (id in versions) {
//             if (versions[id]["aliases"].length > 0 && versions[id]["aliases"].includes("latest")) {
//                 latest_version = "/" + versions[id].version + "/";
//             }
//         }
//         if (!window.location.pathname.includes("/latest/") && (latest_version.length > 0 && !window.location.pathname.includes(latest_version))) {
//             document.querySelector("div[data-md-component=announce]").innerHTML = "<div id='announce-msg'>This documentation is for a previous version. We recommend you <a href='/latest/'>use the latest</a>.</div>";
//         }
//     };
//     xhr.send();
// });
// Show warning banner when displaying old content versions
window.addEventListener("DOMContentLoaded", function() {
    document.querySelector("div[data-md-component=announce]").innerHTML = "<div id='announce-msg'>Stretch 4 docs are coming soon! Check out the <a target=\"_blank\" href=\"https://github.com/hello-robot?q=stretch4_&type=&language=&sort=\">open source repositories</a> on GitHub!</div>";
});


