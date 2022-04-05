(function () {
    "use strict";

    let fs = require("fs");
    let branches = require("./branches.json");

    fs.writeFileSync("minbranches.json", JSON.stringify(branches), "utf-8");

})();