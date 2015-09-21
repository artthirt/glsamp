#include "simple_xml.hpp"

#include <QFile>

SimpleXML::SimpleXML(const QString& filename, bool forSave, const QString &tag)
{
	m_fileName = filename;
	if(forSave)
		if(!load()){
			create_processing_instruction();
			tree_node = create_tree(tag);
		}else{
			tree_node = dom.elementsByTagName("tree").item(0);
		}
	else{
	}
}

/// \brief load from file
bool SimpleXML::load()
{
	QFile f_xml(m_fileName);
	if(!f_xml.open(QIODevice::ReadOnly)){
		return false;
	}

	QString str;
	int el, ec;
	dom.setContent((QIODevice*)&f_xml, &str, &el, &ec);
	f_xml.close();
	return true;
}

/// \brief save to file
void SimpleXML::save()
{
	QString xml = dom.toString();
	QFile f_xml(m_fileName);
	if(f_xml.open(QIODevice::WriteOnly)){
		f_xml.write(xml.toUtf8());
		f_xml.close();
	}
}

/// \brief create <?xml version="1.0"?>
void SimpleXML::create_processing_instruction()
{
	QDomProcessingInstruction el = dom.createProcessingInstruction("xml", "version='1.0' encoding='UTF-8'");
	dom.appendChild(el);
}

/// \brief create node for '_dom' and return 'node'
QDomNode SimpleXML::create_tree(QDomNode& _dom, QDomNode& node, const QString& tag)
{
	QDomElement el = dom.createElement(tag);
	node = _dom.appendChild(el);
	return node;
}

/**
 * @brief create_tree
 * @param tag
 * @return
 */
QDomNode SimpleXML::create_tree(const QString& tag)
{
	QDomNode node;
	create_tree(dom, node, tag);
	return node;
}

/// \brief create tag with text = string value
void SimpleXML::set_dom_value_s(QDomNode& node, const QString& tag, const QString& value)
{
	set_tag_value(tag, value, &node);
}

void SimpleXML::set_dom_value_s(const QString &tag, const QString &value)
{
	set_tag_value(tag, value);
}

/// \brief create tag with text = unsigned integer value
void SimpleXML::set_dom_value_num(QDomNode& node, const QString& tag, double value)
{
	set_tag_value(tag, QString::number(value), &node);
}

void SimpleXML::set_dom_value_num(const QString &tag, double value)
{
	set_tag_value(tag, QString::number(value));
}

/// \brief прочитать строку из xml
QString SimpleXML::get_xml_string(const QString tag, QDomNode* node)
{
	QDomNodeList list;
	if(!node){
		list = dom.elementsByTagName(tag);
	}else{
		list = node->toElement().elementsByTagName(tag);
	}
	if(list.size()){
		QDomNode value = list.item(0);
		QDomText dtext = value.firstChild().toText();
		QString text = dtext.data();
		return text;
	}
	return "";
}

/// \brief read unsigned integer value from tag
uint SimpleXML::get_xml_uint(const QString tag, QDomNode* node)
{
	return  get_xml_string(tag, node).toUInt();
}

/// \brief read integer value from tag
int SimpleXML::get_xml_int(const QString tag, QDomNode* node)
{
	return get_xml_string(tag, node).toInt();
}

/// \brief read double value from tag
double SimpleXML::get_xml_double(const QString tag, QDomNode* node)
{
	return  get_xml_string(tag, node).toDouble();
}

/// \brief get list attributes
QDomNamedNodeMap SimpleXML::get_list_attributes(const QString tag, QDomNode* node)
{
	QDomNodeList list;
	if(!node){
		list = dom.elementsByTagName(tag);
	}else{
		list = node->toElement().elementsByTagName(tag);
	}
	if(list.size()){
		QDomNode value = list.item(0);
		return value.attributes();
	}
	return QDomNamedNodeMap();
}

/// \brief read unsigned integer value from tag
uint SimpleXML::get_xml_uint(QDomNamedNodeMap node, const QString& tag)
{
	return  node.namedItem(tag).nodeValue().toUInt();
}

/// \brief read integer value from tag
int SimpleXML::get_xml_int(QDomNamedNodeMap node, const QString& tag)
{
	return node.namedItem(tag).nodeValue().toInt();
}

/// \brief read double value from tag
double SimpleXML::get_xml_double(QDomNamedNodeMap node, const QString& tag)
{
	return  node.namedItem(tag).nodeValue().toDouble();
}
/**
 * @brief first_child
 * @return
 */
QDomNode SimpleXML::first_child()
{
	return dom.firstChild();
}

/// \brief get node list form xml
QDomNodeList& SimpleXML::get_xml_list(const QString node)
{
	dom_list = dom.elementsByTagName(node);
	return dom_list;
}

/// \brief get count nodes
int SimpleXML::count_from_list()
{
	return dom_list.size();
}

/// \brief get node form list
QDomNode SimpleXML::get_xml_node_from_list(int index)
{
	return dom_list.item(index);
}

/// \brief read int from node of list
int SimpleXML::get_xml_int(int index)
{
	QDomNode value = dom_list.item(index);
	QDomText dtext = value.firstChild().toText();
	QString text = dtext.data();
	return text.toUInt();
}

/// \brief read string from node of list
QString SimpleXML::get_xml_string(int index)
{
	QDomNode value = dom_list.item(index);
	QDomText dtext = value.firstChild().toText();
	QString text = dtext.data();
	return text;
}

QDomNode SimpleXML::get_node(QDomNode &parent_node, const QString &tag)
{
	QDomNode node;
	if(parent_node.isNull())
		node = dom.elementsByTagName(tag).item(0);
	else{
		node = parent_node.toElement().elementsByTagName(tag).item(0);
	}
	if(node.isNull()){
		QDomElement el = dom.createElement(tag);
		if(parent_node.isNull()){
			node = dom.appendChild(el);
		}else{
			node = parent_node.appendChild(el);
		}
	}
	return node;
}

void SimpleXML::set_tag_value(const QString &tag, const QString &value, QDomNode* parent, int index)
{
	QDomNode node;
	if(parent)
		node = parent->toElement().elementsByTagName(tag).item(index);
	else
		node = dom.elementsByTagName(tag).item(index);
	if(node.isNull()){
		QDomElement el = dom.createElement(tag);
		QDomNode _n = dom.appendChild(el);
		QDomText _t = dom.createTextNode(value);
		_n.appendChild(_t);
		if(!parent)
			tree_node.appendChild(_n);
		else
			parent->appendChild(_n);
	}else{
		if(node.firstChild().isNull()){
			QDomText _t = dom.createTextNode(value);
			node.appendChild(_t);
		}else
			node.firstChild().setNodeValue(value);
	}
}
