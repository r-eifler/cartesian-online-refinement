# -*- coding: utf-8 -*-

from collections import defaultdict

from external import txt2tags


        
def _get_config(target):
    
    config = {}
    
    # Set the configuration on the 'config' dict.
    config = txt2tags.ConfigMaster()._get_defaults()
    
    # The Pre (and Post) processing config is a list of lists:
    # [ [this, that], [foo, bar], [patt, replace] ]
    config['postproc'] = []
    config['preproc'] = []
    
    
    if target in ['xhtml', 'html']:
        config['encoding'] = 'UTF-8'       # document encoding
        config['toc'] = 0
        config['css-inside'] = 1
        config['css-sugar'] = 1
        
        # Allow line breaks, r'\\\\' are 2 \ for regexes
        config['postproc'].append([r'\\\\', '<br />'])
        
    elif target == 'texo':
        config['encoding'] = 'utf8'
        config['preproc'].append(['€', 'Euro'])
        
        # Latex only allows whitespace and underscores in filenames if
        # the filename is surrounded by "...". This is in turn only possible
        # if the extension is omitted
        config['preproc'].append([r'\[""', r'["""'])
        config['preproc'].append([r'""\.', r'""".'])
        
        # For images we have to omit the file:// prefix
        config['postproc'].append([r'includegraphics\{(.*)"file://', r'includegraphics{"\1'])
        #config['postproc'].append([r'includegraphics\{"file://', r'includegraphics{"'])
        
        # Allow line breaks, r'\\\\' are 2 \ for regexes
        config['postproc'].append([r'\$\\backslash\$\$\\backslash\$', r'\\\\'])
        
    elif target == 'txt':
        # Allow line breaks, r'\\\\' are 2 \ for regexes
        config['postproc'].append([r'\\\\', '\n'])
        
    return config
    
    
    
class Document(object):
    #TODO: Allow pre-/postprocessing
    def __init__(self, title='', author='', date=''):
        self.title = title
        self.author = author
        self.date = date
        
        self.text = ''
        
        
    def add_text(self, text):
        self.text += text + '\n'


    def render(self, target, options=None):
        #res = '\n'.join([self.title, self.author, self.date]) + '\n\n'
        
        # Bug in txt2tags: Titles are not escaped
        if target == 'tex':
            self.title = self.title.replace('_', r'\_')
        
        # Here is the marked body text, it must be a list.
        txt = self.text.split('\n')
        
        # Set the three header fields
        headers = [self.title, self.author, self.date]
        
        config = _get_config(target)
        
        config['outfile'] = txt2tags.MODULEOUT  # results as list
        config['target'] = target
        
        if options is not None:
            config.update(options)
        
        # Let's do the conversion
        try:
            headers   = txt2tags.doHeader(headers, config)
            body, toc = txt2tags.convert(txt, config)
            footer  = txt2tags.doFooter(config)
            toc = txt2tags.toc_tagger(toc, config)
            toc = txt2tags.toc_formatter(toc, config)
            full_doc  = headers + toc + body + footer
            finished  = txt2tags.finish_him(full_doc, config)
            result = '\n'.join(finished)
        
        # Txt2tags error, show the messsage to the user
        except txt2tags.error, msg:
            logging.error(msg)
            result = msg
        
        # Unknown error, show the traceback to the user
        except:
            result = txt2tags.getUnknownErrorMessage()
            logging.error(result)
        
        return result


