(function () {
    "use strict";

    let fs = require("fs");
    let branches = require("./branches.json");

    fs.writeFileSync("minbranches.json", JSON.stringify(branches), "utf-8");

    /*let inoFile;
    inoFile = fs.readFileSync("../panlanshield.ino", "utf8");

    let regex = /const char \*jsonFile =.*\n/
    inoFile.replace(regex, `const char *jsonFile = "${JSON.stringify(branches)}";\n`);

    fs.writeFileSync("../panlanshield.ino.test", inoFile, "utf8");*/

})();