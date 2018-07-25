const exportHTMLToPDF = async (filename, page_divs, header) => {
    const opt = {
        margin: [0,0],
        image: { type: 'jpeg', quality: 0.98 },
        html2canvas: { dpi: 300, letterRendering: true },
        jsPDF: { unit: 'mm', format: 'a4', orientation: 'portrait' }
    };
    const doc = new jsPDF(opt.jsPDF);
    // const pageSize = jsPDF.getPageSize(opt.jsPDF);
    for(let i = 0; i < page_divs.length; i++){
        const page = page_divs[i];
        const pageImage = await html2pdf().from(page).set(opt).outputImg();
    
        if(i != 0) {
            doc.addPage();
        }
        doc.addImage(pageImage.src, 'jpeg', opt.margin[0], opt.margin[1]+15, doc.internal.pageSize.width, doc.internal.pageSize.height);

        // add header
        const headerImage = await html2pdf().from(header).set(opt).outputImg();
        doc.addImage(headerImage.src, 'jpeg', opt.margin[0]+10, opt.margin[1]-5, doc.internal.pageSize.width-20, 25);
    }
    // This can be whatever output you want. I prefer blob.
    const pdf = doc.save(filename);
}


function GeneratePDF(filename){
    console.log(filename);

    // remove elements
    elementsToRemove = document.getElementsByClassName('pdf-exclude')
    while(elementsToRemove[0]){
        elementsToRemove[0].parentNode.removeChild(elementsToRemove[0]);
    }

    var pages = document.getElementsByClassName('page-wrapper');
    var header = document.getElementById('header');
    pdf = exportHTMLToPDF(filename, pages, header);
}


function CreateDocumentToExportPDF(filename){
                    
    // get handle to pdf iFrame
    var pdf_iFrame = document.getElementById("pdf_iframe");
    if(pdf_iFrame == null){
        // create pdf iframe 
        var pdf_iFrame = document.createElement('iframe');
        pdf_iFrame.id = "pdf_iframe";

        // used to hide the iframe
        pdf_iFrame.style.position = "absolute";
        pdf_iFrame.style.top = "-1000000px";

        // append to document body
        document.body.appendChild(pdf_iFrame);
    }

    // get the contents of this page to be inserted into the iFrame
    var contents = document.getElementById("main").innerHTML;

    // get handle to document within the pdf iFrame
    pdf_iFrame_doc = pdf_iFrame.contentWindow.document;

    // open pdf iframe doc
    // include print style sheet css and required javascript 
    // write content want to export to pdf
    pdf_iFrame_doc.open();
    pdf_iFrame_doc.write('<html><head><title>PDF Document</title>');
    pdf_iFrame_doc.write('<link rel="stylesheet" href="assets/css/print.css" type="text/css"/>');
    pdf_iFrame_doc.write('<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/es6-promise@4/dist/es6-promise.auto.js"><\/script>');
    pdf_iFrame_doc.write('<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/jspdf/1.3.4/jspdf.debug.js"><\/script>');
    pdf_iFrame_doc.write('<script type="text/javascript" src="assets/js/html2canvas.min.js"><\/script>');
    pdf_iFrame_doc.write('<script type="text/javascript" src="assets/js/html2pdf.min.js"><\/script>');
    pdf_iFrame_doc.write('<script type="text/javascript" src="assets/js/generatePDF.js"><\/script>');
    pdf_iFrame_doc.write('<script type="text/javascript">');
    pdf_iFrame_doc.write('window.onload = function(){GeneratePDF(\''+filename+'\');}')
    pdf_iFrame_doc.write('<\/script>');
    pdf_iFrame_doc.write('</head><body>');
    pdf_iFrame_doc.write(contents);
    pdf_iFrame_doc.write('</body></html>');
    pdf_iFrame_doc.close();
    
}